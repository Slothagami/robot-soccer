var canv, c, fps = 24
var spaceCanv, sc, filledBg = false
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

var ball = {x: 200, y:200},
    speed = 5,
    mouse = {x:0, y:0},
    minDist = 5,
    botSize = 100,
    cell = 9

function next_point(point, func, origin=undefined, color="white") {
    let dist = Math.hypot(ball.x - point.x, ball.y - point.y)
    if(dist < minDist) return
    if(abs(point.x) > 1000 || abs(point.y) > 1000) return

    let angle = Math.atan2(ball.y - point.y, ball.x - point.x)
    let dx = Math.cos(func(angle)) * speed
    let dy = Math.sin(func(angle)) * speed
    let npoint = {x: point.x + dx, y: point.y + dy}
    let delta = {x: npoint.x - point.x, y: npoint.y - point.y}

    // Draw the point
    c.beginPath()
    c.strokeStyle = color
    if(delta.x < 0 && dist < botSize) {
        c.strokeStyle = "red"

        sc.fillStyle = "#300000"
        if(origin != undefined) {
            sc.fillRect(origin.x, origin.y, cell, cell)
            return
        }
    }
    c.moveTo(point.x, point.y)
    c.lineTo(npoint.x, npoint.y)
    c.stroke()

    next_point(npoint, func, origin, color)
}

function fillBg() {
    for(let x = 0; x < canv.width; x += cell) {
        for(let y = 0; y < canv.height; y += cell) {
            next_point({x: x, y: y}, ANGLE_FUNC, {x: x, y: y})
        }
    }
}

function init() {
    ball.x = canv.width  / 2
    ball.y = canv.height / 2

    c.strokeStyle = "white"

    window.addEventListener("mousemove", e => {
        mouse.x = e.x
        mouse.y = e.y
    })
    window.addEventListener("keydown", e => {
        if(e.code == "Space" && !filledBg) {
            fillBg()
            filledBg = true
        }
    })

    // make output space canvas
    spaceCanv = document.createElement("canvas")
    sc = spaceCanv.getContext("2d")

    spaceCanv.width  = canv.width
    spaceCanv.height = canv.height
    document.body.appendChild(spaceCanv)

}

function main() {
    c.clearRect(0,0, canv.width, canv.height)

    // draw other canvas
    c.drawImage(spaceCanv, 0, 0)

    // Draw bot
    c.beginPath()
    c.strokeStyle = "#666"
    c.arc(mouse.x, mouse.y, botSize, 0, Math.PI * 2)
    c.stroke()

    // Draw Path
    next_point(mouse, ANGLE_FUNC)
    next_point(mouse, COMPARE_FUNC, undefined, "#666")

    // Draw Ball
    c.beginPath()
    c.fillStyle = "white"
    c.arc(ball.x, ball.y, 8, 0, Math.PI * 2)
    c.fill()
}
