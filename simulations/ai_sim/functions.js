const radians = degrees => degrees * Math.PI/180

function squareNorm(x, y) {
    scale = 1 / Math.max(Math.abs(x), Math.abs(y))
    return {
        x: x * scale,
        y: y * scale
    }
}
