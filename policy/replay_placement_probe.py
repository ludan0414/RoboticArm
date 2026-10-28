"""Automatically replay a successful placement-only probe; no Pick buttons."""
import argparse
import json
from pathlib import Path
from replay_prepared_demo import replay


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle',type=Path,required=True)
    parser.add_argument('--fps',type=int,default=30)
    parser.add_argument('--device',default='cuda:0')
    args=parser.parse_args()
    result=json.loads((args.bundle/'result.json').read_text(encoding='utf-8'))
    if not result.get('success') or result.get('scope')!='pick_carry_place_only':
        raise ValueError('placement-only probe has not completed successfully')
    if args.fps <= 0: raise ValueError('fps must be positive')
    print('AUTOMATIC PLACEMENT REPLAY: empty returns omitted; starts teleport between cases',flush=True)
    replay(args.bundle,device=args.device,fps=args.fps,manual_pick=False,unvalidated_preview=True)

if __name__=='__main__':main()
