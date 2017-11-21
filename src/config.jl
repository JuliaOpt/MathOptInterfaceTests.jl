struct TestConfig
    atol::Float64 # absolute tolerance for ...
    rtol::Float64 # relative tolerance for ...
    query::Bool # can get objective function, and constraint functions, and constraint sets
    duals::Bool # test dual solutions
    infeas_certificates::Bool # check for infeasibility certificates when appropriate
end

"""
    @moitestset setname subsets

Defines a function `setnametest(solver, config, exclude)` that runs the tests defined in the dictionary `setnametests`
with the solver `solver` and config `config` except the tests whose dictionary key is in `exclude`.
If `subsets` is `true` then each test runs in fact multiple tests hence the `exclude` argument is passed
as it can also contains test to be excluded from these subsets of tests.
"""
macro moitestset(setname, subsets=false)
    testname = Symbol(string(setname) * "test")
    testdict = Symbol(string(testname) * "s")
    if subsets
        runtest = :( f(solver, config, exclude) )
    else
        runtest = :( f(solver, config) )
    end
    :(
        function $testname(solver::Function, config::TestConfig, exclude::Vector{String} = String[])
            for (name,f) in $testdict
                if name in exclude
                    continue
                end
                @testset "$name" begin
                    $runtest
                end
            end
        end
    )
end
