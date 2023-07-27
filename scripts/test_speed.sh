#!/bin/bash
velocity="1.2"
limit="1.2"
increment=".2"

while [ "$(bc <<< "$velocity <= $limit" )" == "1" ] ; do
    for j in {1..3} ; do
        # run the test
        python3 run_sims_speed.py --bag --max_speed "$velocity"
        sleep 30
    done

    velocity=$(bc <<< "$velocity+$increment")
done
