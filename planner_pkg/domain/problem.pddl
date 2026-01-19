(define (problem assignment2_exploration)
    (:domain assignment2)

    (:objects
        mogi_bot - robot
        wp_start wp1 wp2 wp3 wp4 - waypoint
    )

    (:init
        (robot_at mogi_bot wp_start)
    )

    (:goal 
        (and 
            (searched wp1)
            (searched wp2)
            (searched wp3)
            (searched wp4)
        )
    )
)