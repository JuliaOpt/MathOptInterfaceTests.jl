struct TestConfig
    atol::Float64 # absolute tolerance for ...
    rtol::Float64 # relative tolerance for ...
    query::Bool # can get objective function, and constraint functions, and constraint sets
    duals::Bool # test dual solutions
    infeas_certificates::Bool # check for infeasibility certificates when appropriate
end

macro moitestset(setname)
    testname = Symbol(string(setname) * "test")
    testdict = Symbol(string(testname) * "s")
    :(
        function $testname(solver::MOI.AbstractSolver, config::TestConfig, exclude::Vector{String} = String[])
            for (name,f) in $testdict
                if name in exclude
                    continue
                end
                @testset "$name" begin
                    f(solver, config, exclude)
                end
            end
        end
    )
end
