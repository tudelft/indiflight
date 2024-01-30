#!/usr/bin/env python3
from indiflight_log_importer import IndiflightLog
import logging

if __name__=="__main__":
    logging.basicConfig(
        format='%(asctime)s -- %(name)s %(levelname)s: %(message)s',
        level=logging.INFO,
        )

    # import data
    real = IndiflightLog("/mnt/data/WorkData/BlackboxLogs/2024-01-19/CyberzooTests/LOG00521_catapultAndSequence.BFL",
                        (2179, 2677))
    sim = IndiflightLog("/mnt/data/WorkData/BlackboxLogs/2024-01-19/LOG00503_CineRate_HIL_CorrectSequence.BFL",
                        (2179, 2677))

    # plot some stuff
    real.compare(sim, self_name="Real", other_name="Sim")
