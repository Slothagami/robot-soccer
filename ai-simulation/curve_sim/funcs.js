// Utility
const lerp = (a, b, amm) => (b - a) * amm + a
const pi   = Math.PI 
const abs  = Math.abs

// Angle Functions
const double    = angle => angle * 2
const lerpScale = angle => angle * lerp(2, 1.3, abs(angle / pi))
const log       = angle => angle * Math.log(abs(angle) + pi/2)
const exp       = angle => Math.exp(-angle+1)

const map = perc => perc ** 2

function line(angle) {
    let perc = abs(angle / pi)
    if (perc > .7) return lerpScale(angle)
    return angle * 1.75//1.9
}

// Tradeoff seems to be: slimmer, more space efficient path eg(1.3, 2.5, .7),
// or wider, better ball catching path eg(1.8, 2, 1.4)
let back = 1.8, peak = 2, front = 1.4//1.3?
function beasier(angle) {
    // smooth extension of line()
    let perc = abs(angle / pi)
    return angle * lerp(back, lerp(peak, front, perc), perc)
}
function beasier2(angle) {
    // smooth extension of line()
    let perc = abs(angle / pi)
    if (perc > .7) {
        return angle * lerp(back, lerp(peak, front, perc), perc)
    }
    return lerpScale(angle)
}

const ANGLE_FUNC = line
const COMPARE_FUNC = lerpScale
