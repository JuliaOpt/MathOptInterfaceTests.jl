module MathOptInterfaceTests

using MathOptInterface
const MOI = MathOptInterface

using Base.Test

include("config.jl")

include("modellike.jl")

include("contlinear.jl")
include("contconic.jl")
include("contquadratic.jl")

include("intlinear.jl")
include("intconic.jl")

end # module
