var goal = {
    width:  .06,
    height: .42
}

function drawFieldLines() {
    c.fillStyle = "#ffffff11"

    // Goals
    let goalY = canv.height * (1-goal.height)/2,
        goalWidth  = canv.width  * goal.width,
        goalHeight = canv.height * goal.height

    c.fillRect(0, goalY, goalWidth, goalHeight)
    c.fillRect(canv.width - goalWidth, goalY, goalWidth, goalHeight)

    // Border
    let margin = goalWidth + 2
    c.strokeStyle = "#ffffff08"
    c.strokeRect(margin, margin, canv.width-2*margin, canv.height-2*margin)
}