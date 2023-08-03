define laser ranger
(
    sensor(
        ranger [ 0.0 5.6]
        fov 240.0
        samples 240
    )
    size [ 0.06 0.06 0.06]
    color "blue"

    block( points 4
        point[0] [0 0]
        point[1] [0 1]
        point[2] [1 1]
        point[3] [1 0]
        z [0 1]
    )
   
)



define simple_robot position
(
      size [1 1 1]

      # the shape of Bigbob

      block
      (
            points 8
            point[7] [0.25 0.1]
            point[6] [0 0.35]
            point[5] [0 0.85]
            point[4] [0.25 1.1]
            point[3] [0.75 1.1]
            point[2] [1 0.85]
            point[1] [1 0.35]
            point[0] [0.75 0.1]
            z [0 1]
      )

      block
      (
            points 4
            point[3] [0.375 1.2]
            point[2] [0.625 1.2]
            point[1] [0.625 1.1]
            point[0] [0.375 1.1]
            z [0 0.25]
      )

      block
      (
            points 4
            point[3] [0.375 0.1]
            point[2] [0.625 0.1]
            point[1] [0.625 0]
            point[0] [0.375 0]
            z [0 0.25]
      )
      odom_error [0.03 0.03 0.00 0.05]
      localization "odom"
      gui_nose 1
      drive "diff"
      color "red"
      laser(pose [ 0.6 0.0 -0.15 0.0])
)
