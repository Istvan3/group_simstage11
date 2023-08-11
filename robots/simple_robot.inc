define laser ranger(
    sensor(
        ranger [ 0.0 5.6]
        fov 240.0
        samples 240
    )
    color "white"
    size [ 0.06 0.06 0.06 ] #Size of Pac-man's food (tiny dot)
    block( points 4
        point[0] [2 1]
        point[1] [3 1]
        point[2] [3 2]
        point[3] [2 2]
        z [0 1]
    )
)

define simple_robot position(
    odom_error [0.03 0.03 0.00 0.05]
    localization "odom"
    size [ 1 1 1 ]
    gui_nose 1
    drive "diff"
    color "yellow"
    laser(pose [ 0.23 0.0 -0.15 0.0])
    #Pac-Man shape definition: 
    block(
        points 28
        point[0] [1 0]
        point[1] [2 0]
        point[2] [2 0.5]
        point[3] [2.5 0.5]
        point[4] [2.5 1]
        point[5] [2 1]
        point[6] [2 1.25]
        point[7] [1.5 1.25]
        point[8] [1.5 1.375]
        point[9] [1.25 1.375]
        point[10] [1.25 1.625]
        point[11] [1.5 1.625]
        point[12] [1.5 1.75]
        point[13] [2 1.75]
        point[14] [2 2]
        point[15] [2.5 2]
        point[16] [2.5 2.5]
        point[17] [2 2.5]
        point[18] [2 3]
        point[19] [1 3]
        point[20] [1 2.5]
        point[21] [0.5 2.5]
        point[22] [0.5 2]
        point[23] [0 2]
        point[24] [0 1]
        point[25] [0.5 1]
        point[26] [0.5 0.5]
        point[27] [1 0.5]
        color "yellow"
        size [1 1 1]  # Pac-man size
        z [0 1]
    )
)
