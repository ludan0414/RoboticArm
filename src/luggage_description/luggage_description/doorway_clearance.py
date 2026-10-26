"""ROS-free conservative front-frame clearance geometry.

The four inflated front-frame prisms enforce a margin around the rectangular
aperture, including rotation of an attached payload. They do not model an
unmeasured movable door leaf. The physical container descriptor stays unchanged.
"""
from dataclasses import dataclass
import math

from .container_geometry import descriptor_from_scene_config, normalize_descriptor

MIN_SITE_CLEARANCE_M = .05
ALIGNMENT_SLACK_M = .005


def vector(values, length, name):
    if not isinstance(values, (list, tuple)) or len(values) != length:
        raise ValueError(name+' has wrong dimensions')
    result=tuple(float(v) for v in values)
    if not all(math.isfinite(v) for v in result):
        raise ValueError(name+' must be finite')
    return result


def rotation_matrix(quaternion):
    x,y,z,w=vector(quaternion,4,'quaternion')
    norm=math.sqrt(x*x+y*y+z*z+w*w)
    if abs(norm-1)>.01:
        raise ValueError('doorway transform requires a unit quaternion')
    x,y,z,w=(v/norm for v in (x,y,z,w))
    return ((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),
            (2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),
            (2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)))


def rotate(matrix, point):
    return tuple(sum(row[j]*point[j] for j in range(3)) for row in matrix)


@dataclass(frozen=True)
class DoorwayGeometry:
    descriptor: dict
    outer_size: tuple
    aperture: tuple
    clearance: float

    @classmethod
    def from_scene(cls, scene):
        container=scene['container']
        opening=container['opening']
        if opening['side']!='negative_x':
            raise ValueError('only negative_x rectangular doorways are supported')
        return cls.from_settings(dict(schema_version=1,container_geometry=descriptor_from_scene_config(scene).descriptor(),
            outer_size_m=[container['outer'][key] for key in ('length','width','height')],
            aperture_corners_m=opening['aperture']['corners'],clearance_m=opening.get('clearance_m',0.0)))

    @classmethod
    def from_settings(cls, settings, require_site_margin=False):
        if settings.get('schema_version')!=1:
            raise ValueError('doorway_clearance schema_version=1 required')
        geometry=normalize_descriptor(settings['container_geometry'])
        if geometry.frame_id!='container_link':
            raise ValueError('doorway requires container_link geometry')
        outer=vector(settings['outer_size_m'],3,'outer_size_m')
        corners=tuple(vector(p,3,'aperture corner') for p in settings['aperture_corners_m'])
        if len(corners)!=4 or len(set(corners))!=4:
            raise ValueError('four distinct aperture corners required')
        xs,ys,zs=({p[i] for p in corners} for i in range(3))
        if len(xs)!=1 or len(ys)!=2 or len(zs)!=2 or abs(next(iter(xs))+geometry.half_x)>1e-6:
            raise ValueError('aperture must be a rectangle on the negative_x face')
        margin=float(settings['clearance_m'])
        if not math.isfinite(margin) or margin<0 or (require_site_margin and margin<MIN_SITE_CLEARANCE_M):
            raise ValueError('site doorway clearance must be at least 0.05 m')
        if (outer[0]<geometry.length or outer[1]<geometry.width or outer[2]<geometry.ceiling_z
                or min(ys)<-geometry.half_y or max(ys)>geometry.half_y
                or min(zs)<geometry.floor_z or max(zs)>geometry.ceiling_z
                or min(max(ys)-min(ys),max(zs)-min(zs))<=2*(margin+ALIGNMENT_SLACK_M)):
            raise ValueError('invalid outer/aperture/clearance geometry')
        return cls(geometry.descriptor(),outer,corners,margin)

    def settings(self):
        return dict(schema_version=1,container_geometry=self.descriptor,outer_size_m=list(self.outer_size),
                    aperture_corners_m=[list(p) for p in self.aperture],clearance_m=self.clearance)

    @property
    def bounds(self):
        return (self.aperture[0][0],min(p[1] for p in self.aperture),max(p[1] for p in self.aperture),
                min(p[2] for p in self.aperture),max(p[2] for p in self.aperture))

    def local_guard_boxes(self, collision_tolerance_m=0.0):
        if self.clearance==0:
            return []
        margin=self.clearance+float(collision_tolerance_m)
        if not math.isfinite(margin) or collision_tolerance_m<0:
            raise ValueError('invalid collision tolerance')
        x,y0,y1,z0,z1=self.bounds
        length,width,height=self.outer_size
        # Inflate the physical front frame in all axes. This protects the
        # finite front thickness, rather than checking only a centre plane.
        slabs=[('left',( -length/2,y1,0),(x,width/2,height)),
               ('right',(-length/2,-width/2,0),(x,y0,height)),
               ('bottom',(-length/2,-width/2,0),(x,width/2,z0)),
               ('top',(-length/2,-width/2,z1),(x,width/2,height))]
        result=[]
        for name,lower,upper in slabs:
            size=[upper[i]-lower[i]+2*margin for i in range(3)]
            if min(size)<=0:
                raise ValueError('invalid doorway frame thickness')
            result.append(dict(id='site_door_clearance_'+name,
                xyz=[(lower[i]+upper[i])/2 for i in range(3)],size=size))
        return result

    def world_guard_boxes(self, container_xyz, container_quat, collision_tolerance_m=0.0):
        origin=vector(container_xyz,3,'container position')
        quaternion=vector(container_quat,4,'container quaternion')
        matrix=rotation_matrix(quaternion)
        result=[]
        for box in self.local_guard_boxes(collision_tolerance_m):
            offset=rotate(matrix,box['xyz'])
            result.append(dict(box,xyz=[origin[i]+offset[i] for i in range(3)],quat=list(quaternion)))
        return result

    def align_box(self, position, quaternion, size, container_xyz, container_quat):
        """Align an entire oriented box while it overlaps the protected frame.

        Returns a shifted WORLD centre. Orientation is never altered. Exact
        trajectory collision checking against the guard prisms remains required.
        """
        origin=vector(container_xyz,3,'container position')
        center=vector(position,3,'box position')
        dimensions=vector(size,3,'box dimensions')
        if min(dimensions)<=0:
            raise ValueError('positive box dimensions required')
        rotation=rotation_matrix(container_quat)
        inverse=tuple(zip(*rotation))
        local=rotate(inverse,tuple(center[i]-origin[i] for i in range(3)))
        box_rotation=rotation_matrix(quaternion)
        relative=tuple(tuple(sum(inverse[i][k]*box_rotation[k][j] for k in range(3)) for j in range(3)) for i in range(3))
        extent=tuple(sum(abs(relative[i][j])*dimensions[j]/2 for j in range(3)) for i in range(3))
        x,y0,y1,z0,z1=self.bounds
        margin=self.clearance+ALIGNMENT_SLACK_M+.001
        if local[0]+extent[0]<-self.outer_size[0]/2-margin or local[0]-extent[0]>x+margin:
            return center
        aligned=list(local)
        for axis,lower,upper in ((1,y0,y1),(2,z0,z1)):
            minimum,maximum=lower+margin+extent[axis],upper-margin-extent[axis]
            if minimum>maximum:
                raise ValueError('DOORWAY_PAYLOAD_TOO_LARGE for configured clearance')
            aligned[axis]=min(max(aligned[axis],minimum),maximum)
        shifted=rotate(rotation,aligned)
        return tuple(origin[i]+shifted[i] for i in range(3))
