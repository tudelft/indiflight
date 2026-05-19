
import pandas as pd
from argparse import ArgumentParser

exclude = [
    "LOG00007",
    "LOG00009",
    "LOG00024",
    "LOG00025",
]

if __name__ == "__main__":
    parser = ArgumentParser(description="Load RLS sim results from .csv and analyze")
    parser.add_argument("csv", type=str, nargs='+')
    parser.add_argument("--model", type=str)

    args = parser.parse_args()

    dfs = None
    for csv in args.csv:
        df = pd.read_csv(csv)
        df['csv'] = csv.split('/')[-2]

        if args.model is not None:
            df = df[df['model'] == args.model].copy()

        if dfs is None:
            dfs = df.copy()
        else:
            dfs = pd.concat((dfs, df), ignore_index=True)

    dfs_excluded = dfs.loc[[l not in exclude for l in dfs['logfile']]]

    summary_excluded = dfs_excluded.groupby('csv').agg({'param_rmse_controller': ["mean", "std"]})
    summary = dfs.groupby('csv').agg({'param_rmse_controller': ["mean", "std"]})

