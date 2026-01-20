(define (domain marker_finder)
(:requirements :strips :typing :durative-actions)

(:types
    robot
    waypoint
    marker
)

(:predicates
    (robot_at ?r - robot ?w - waypoint)
    (marker_at ?m - marker ?w - waypoint)
    (searched ?w - waypoint)      ; Phase 1: Waypoint hint visited
    (photo_taken ?m - marker)     ; Phase 2: Marker processed
)

;; Used for both moving to waypoints (Phase 1) and markers (Phase 2)
(:durative-action move
    :parameters (?r - robot ?w1 ?w2 - waypoint)
    :duration ( = ?duration 5)
    :condition (at start (robot_at ?r ?w1))
    :effect (and
        (at start (not (robot_at ?r ?w1)))
        (at end (robot_at ?r ?w2))
    )
)

;; Phase 1: Triggers the ArUco detection logic
(:durative-action search_waypoint
    :parameters (?r - robot ?w - waypoint)
    :duration ( = ?duration 5)
    :condition (at start (robot_at ?r ?w))
    :effect (at end (searched ?w))
)

;; Phase 2: Triggers the "Take Picture" and image modification logic
(:durative-action take_picture
    :parameters (?r - robot ?m - marker ?w - waypoint)
    :duration ( = ?duration 5)
    :condition (and 
        (at start (robot_at ?r ?w))
        (at start (marker_at ?m ?w))
    )
    :effect (at end (photo_taken ?m))
)
)