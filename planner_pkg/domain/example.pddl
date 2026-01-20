(define (domain assignment2)
    (:requirements :strips :typing :adl :durative-actions :equality)

    (:types
        robot
        waypoint
        marker
    )

    (:predicates
        (robot_at ?r - robot ?wp - waypoint)
        (searched ?wp - waypoint)
        (marker_at ?m - marker ?wp - waypoint)
        (processed ?m - marker)
        (next_id ?m1 ?m2 - marker) ;; Enforces processing order
    )

    ;; Move between any two waypoints
    (:durative-action move
        :parameters (?r - robot ?from ?to - waypoint)
        :duration (= ?duration 10)
        :condition (and
            (at start (robot_at ?r ?from))
        )
        :effect (and
            (at start (not (robot_at ?r ?from)))
            (at end (robot_at ?r ?to))
        )
    )

    ;; Phase 1: Search a waypoint to find markers
    (:durative-action search
        :parameters (?r - robot ?wp - waypoint)
        :duration (= ?duration 5)
        :condition (and
            (over all (robot_at ?r ?wp))
        )
        :effect (and
            (at end (searched ?wp))
        )
    )

    ;; Phase 2: Process marker (Visual Servoing)
    ;; Crucial: Requires robot to be at the waypoint where the marker is.
    (:durative-action process_marker
        :parameters (?r - robot ?m - marker ?wp - waypoint ?prev_m - marker)
        :duration (= ?duration 10)
        :condition (and
            (over all (robot_at ?r ?wp))        ;; during the entire action this ensures the robot didn't move to another waypont
            (over all (marker_at ?m ?wp))      ;; Robot knows m is at wp
            (at start (processed ?prev_m))     ;; Must have processed previous ID
            (at start (next_id ?prev_m ?m))    ;; Enforces the chain order in which the markers should be processed
        )
        :effect (and
            (at end (processed ?m))
        )
    )
)