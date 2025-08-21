var canv, c, fps = 24
window.addEventListener("load", () => {
    canv = document.querySelector("canvas")
    c = canv.getContext("2d")

    const resize = ()=> {
        canv.width  = window.innerWidth
        canv.height = window.innerHeight
    }
    resize()
    document.addEventListener("resize", resize)

    init()
    setInterval(main, 1000 / fps)
})

var mouse = {x:0, y:0}, robot, ball_follow_mouse = false
function init() {
    document.body.addEventListener("mousemove", e => {
        mouse.x = e.x
        mouse.y = e.y
    })
    document.body.addEventListener("contextmenu", e => {
        // move the robot to mouse position (same as right click)
        robot.setPos(mouse.x, mouse.y)
    })
    document.body.addEventListener("keydown", e => {
        if(e.code == "Space") robot.toggle()
        if(e.code == "Enter") ball_follow_mouse = !ball_follow_mouse
    })

    robot = new Robot(AI_test)
    ball  = new Ball()
}

function main() {
    if(FADE_BG) {
        c.fillStyle = "#22222211"
        c.fillRect(0,0, canv.width,canv.height)
    } else {
        c.clearRect(0,0, canv.width,canv.height)
    }

    drawFieldLines()
    robot.main()
    ball.main()
}
