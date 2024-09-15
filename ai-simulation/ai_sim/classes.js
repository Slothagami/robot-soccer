class Robot {
    constructor(ai) {
        this.x = canv.width * .15
        this.y = canv.height / 2

        this.speed = 6

        this.dx = 0
        this.dy = 0

        this.radius = 60

        this.running = false
        this.ai = ai

        this.debugText = ""
        this.vector_count = 0
    }

    add_velocity(dir, speed=1) {
        let dx = Math.cos(dir) * speed,
            dy = Math.sin(dir) * speed

        this.dx += dx
        this.dy += dy

        this.drawVector(dx, dy)
    }

    drawVector(dx, dy, color="auto") {
        if(SHOW_VECTORS) {
            c.beginPath()
            // c.lineWidth = 2
            if(color == "auto") {
                c.strokeStyle = `hsl(${this.vector_count*50}, 60%, 60%)`
            } else {
                c.strokeStyle = color
            }
            c.moveTo(this.x, this.y)
            c.lineTo(
                this.x + dx * this.radius * 2,
                this.y + dy * this.radius * 2
            )
            c.stroke()

            this.vector_count++
        }
    }

    toggle() { this.running = !this.running }

    setPos(x, y) {
        this.x = x
        this.y = y
    }

    move() {
        let norm = squareNorm(this.dx, this.dy)
        let dx = norm.x * this.speed
        let dy = norm.y * this.speed

        this.move_cartesian(dx, dy)

        // Collision with Ball
        let collide_rad = this.radius + ball.radius
        if( dist(ball.x, ball.y, this.x, this.y) < collide_rad) {
            ball.x = this.x + collide_rad
            ball.y = this.y
        }

        this.dx = 0
        this.dy = 0

        this.x = clamp(this.x, this.radius, canv.width  - this.radius)
        this.y = clamp(this.y, this.radius, canv.height - this.radius)
    }

    move_cartesian(dx, dy) {
        this.setPos(this.x + dx, this.y + dy)
    }

    ball_direction() {
        // return angle in range [-pi, pi)
        let angle = Math.atan2(this.y - ball.y, this.x - ball.x) + Math.PI
        if(angle > Math.PI) angle = -(2*Math.PI - angle)
        return angle
    }

    main() {
        this.ai(this)
        if(this.running) this.move()

        // draw circle
        c.fillStyle = "white"
        if(SHOW_VECTORS) c.fillStyle = "#ffffff22"
        c.beginPath()
        c.arc(this.x, this.y, this.radius, 0, 2*Math.PI)
        c.fill()

        // draw angle pointer to ball
        c.strokeStyle = "#999"
        c.lineWidth = 3
        c.textAlign = "center"

        c.beginPath()
        c.moveTo(this.x, this.y)
        let angle = this.ball_direction()
        c.lineTo(
            this.x + Math.cos(angle) * this.radius,
            this.y + Math.sin(angle) * this.radius
        )
        c.stroke()

        this.vector_count = 0

        // draw debug text
        if(!FADE_BG)
            c.fillText(this.debugText, this.x, this.y + this.radius * 2)
    }
}

class Ball {
    constructor() {
        this.radius = 15
        this.x = canv.width  / 2
        this.y = canv.height / 2
    }

    main() {
        // Follow Mouse
        if(ball_follow_mouse) {
            this.x = mouse.x
            this.y = mouse.y
        } else {
            // Draw
            c.fillStyle = "white"
            c.beginPath()
            c.arc(this.x, this.y, this.radius, 0, 2*Math.PI)
            c.fill()
        }

    }
}

// Functions
const clamp = (x,   min,  max) => Math.min(Math.max(x, min), max)
const lerp  = (val, targ, amm) => (targ - val) * amm + val
const dist  = (x1, y1, x2, y2) => Math.hypot(x1 - x2, y1 - y2)
