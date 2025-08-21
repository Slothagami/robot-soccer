const FADE_BG = false
const SHOW_VECTORS = true

const AI = {
    follow_ball: robot => {
        robot.add_velocity(robot.ball_direction())
    },

    intercept: robot => {
        // follow the ball such that it always aproaches from behind,
        // ready to drive it to the goal

        // https://www.desmos.com/calculator/scw2smg0tf
        // use this to see how it behaves under different turn factors "k"
        // lower is better for backwards movement, and higher for forwards

        // interpolate between turn factor based on direction
        let ball_dir = Math.abs(robot.ball_direction())
        let turn
        if(ball_dir > radians(15)){
            let turn_percent = ball_dir / Math.PI
            // let turn  = lerp(2, 1.3, turn_percent)
            turn  = lerp(2, 1.1, turn_percent)
            // max in lerp represents turn when angle == 0, min is turn when angle == pi
        } else {
            turn = 0
        }

        robot.add_velocity(robot.ball_direction() * turn)
    },

    center_align: robot => {
        // push toward center, proportional to distance from it
        let pull_strength = 3
        let dist = -(robot.y - canv.height/2) * pull_strength
        let dist_percent = dist / (canv.height/2)

        let weight = .3
        robot.add_velocity(Math.PI/2, dist_percent * weight)
    },

    main: robot => {
        AI.intercept(robot)
        // AI.center_align(robot)
    }
}
const AI_test = AI.main


/*
    NEEDED INFORMATION FOR AI:
        Direction to ball
        Horizontal Position (relatve to goal)
*/
