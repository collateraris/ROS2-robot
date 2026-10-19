import argparse
from .metrics import read_episodes,report


def main(argv=None):
    p=argparse.ArgumentParser(description='Evaluate episode JSONL using ground truth and real contact samples.')
    p.add_argument('--logs',nargs='+',required=True);p.add_argument('--out',required=True)
    a=p.parse_args(argv);print(report(read_episodes(a.logs),a.out))


if __name__=='__main__':main()
