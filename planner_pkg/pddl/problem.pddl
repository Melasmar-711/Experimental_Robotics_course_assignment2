(define (problem assignment_2_problem)
(:domain marker_finder)

(:objects
    robot1 - robot
    wp0 wp1 wp2 wp3 wp4 - waypoint
)

(:init
    (robot_at robot1 wp0)
    ;; No markers are defined yet because they are unknown 
)

(:goal
    (and
        ;; Phase 1 Goal: All hints visited
        (searched wp1)
        (searched wp2)
        (searched wp3)
        (searched wp4)
        
        ;; Phase 2 Goal: This will be added dynamically by your node
        ;; once the marker IDs are discovered.
    )
)
)