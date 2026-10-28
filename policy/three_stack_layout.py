"""Proposed three-stack geometric arrangement; it is not an IK certificate."""
from online_packing_demo import Box, collides, fully_supported

# Bottom-up per stack. Every label denotes a distinct physical suitcase.
STACKS=(('red','silver','blue'),('pink','smallgrey'),('bronzewhite','black','grey'))
CENTERS=((.60,.28),(.60,.78),(.60,1.28))


def lateral_gap(first, second):
    """Signed separation of two footprints along container-local y."""
    return max(second.pos[1]-(first.pos[1]+first.dims[1]),
               first.pos[1]-(second.pos[1]+second.dims[1]))


def build_three_stacks(inventory, hull, centers=CENTERS):
    lookup={name:(i,dims) for i,(name,dims) in enumerate(zip(inventory['box_labels'],inventory['boxes']))}
    output=[]
    for names,(x,y) in zip(STACKS,centers):
        z=0.0
        for level,name in enumerate(names,1):
            index,dims=lookup[name]
            box=Box('B%d'%(index+1),tuple(dims),(round(x-dims[0]/2,6),round(y-dims[1]/2,6),round(z,6)))
            if (not hull.fits(box) or any(collides(box,other[0]) for other in output)
                    or not (hull.floor_supports(box) or fully_supported(box,[other[0] for other in output]))):
                raise ValueError('invalid geometric support or overlap: '+name)
            output.append((box,index,level))
            z=round(z+dims[2],6)
    return output
