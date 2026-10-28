"""Validate a checked inside-cargo recording against the ground feeder."""
import argparse,json,time
from pathlib import Path
from ground_feeder_bridge import prepare_bridge
from demo_contract import write_json


def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--inside',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
 if args.output.exists():raise FileExistsError(args.output)
 start=time.monotonic()
 try:
  validation=prepare_bridge(args.inside,args.output,[-1.6,0.0],validate=True,
                            observe_at_base=True,observation_base_xy=[-1.2,0.0],allow_partial=True)
  if not validation.get('success'):raise RuntimeError('ground feeder validation failed')
  write_json(args.output/'ground_result.json',dict(success=True,elapsed_sec=time.monotonic()-start,hardware_motion_validated=False))
  print('GROUND_FEEDER_READY',flush=True);return 0
 except BaseException as exc:
  args.output.mkdir(parents=True,exist_ok=True)
  write_json(args.output/'ground_result.json',dict(success=False,error=str(exc),elapsed_sec=time.monotonic()-start,hardware_motion_validated=False))
  print('GROUND_FEEDER_FAILED: %s'%exc,flush=True);return 1
if __name__=='__main__':raise SystemExit(main())
