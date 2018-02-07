# Integer conic problems

function intsoc1test(instance::MOI.AbstractInstance; atol=Base.rtoldefault(Float64), rtol=Base.rtoldefault(Float64))
    #@test MOI.supportsproblem(instance, MOI.ScalarAffineFunction{Float64},
    #    [(MOI.VectorAffineFunction{Float64},MOI.Zeros),
    #     (MOI.SingleVariable,MOI.ZeroOne),
    #     (MOI.VectorOfVariables,MOI.SecondOrderCone)])
    @testset "INTSOC1" begin

        # Problem SINTSOC1
        # min 0x - 2y - 1z
        #  st  x            == 1
        #      x >= ||(y,z)||
        #      (y,z) binary

        MOI.empty!(instance)
        @test MOI.isempty(instance)

        x,y,z = MOI.addvariables!(instance, 3)

        @test MOI.canset(instance, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}())
        MOI.set!(instance, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(), MOI.ScalarAffineFunction([y,z],[-2.0,-1.0],0.0))
        @test MOI.canset(instance, MOI.ObjectiveSense())
        MOI.set!(instance, MOI.ObjectiveSense(), MOI.MinSense)

        @test MOI.canaddconstraint(instance, MOI.VectorAffineFunction{Float64}, MOI.Zeros)
        ceq = MOI.addconstraint!(instance, MOI.VectorAffineFunction([1],[x],[1.0],[-1.0]), MOI.Zeros(1))
        @test MOI.canaddconstraint(instance, MOI.VectorOfVariables, MOI.SecondOrderCone)
        csoc = MOI.addconstraint!(instance, MOI.VectorOfVariables([x,y,z]), MOI.SecondOrderCone(3))

        @test MOI.get(instance, MOI.NumberOfConstraints{MOI.VectorAffineFunction{Float64},MOI.Zeros}()) == 1
        @test MOI.get(instance, MOI.NumberOfConstraints{MOI.VectorOfVariables,MOI.SecondOrderCone}()) == 1
        loc = MOI.get(instance, MOI.ListOfConstraints())
        @test length(loc) == 2
        @test (MOI.VectorAffineFunction{Float64},MOI.Zeros) in loc
        @test (MOI.VectorOfVariables,MOI.SecondOrderCone) in loc

        @test MOI.canaddconstraint(instance, typeof(MOI.SingleVariable(y)), typeof(MOI.ZeroOne))
        bin1 = MOI.addconstraint!(instance, MOI.SingleVariable(y), MOI.ZeroOne)
        @test MOI.canaddconstraint(instance, typeof(MOI.SingleVariable(z)), typeof(MOI.ZeroOne))
        bin2 = MOI.addconstraint!(instance, MOI.SingleVariable(z), MOI.ZeroOne)

        if config.solve
            MOI.optimize!(instance)

            @test MOI.canget(instance, MOI.TerminationStatus())
            @test MOI.get(instance, MOI.TerminationStatus()) == MOI.Success

            @test MOI.canget(instance, MOI.PrimalStatus())
            @test MOI.get(instance, MOI.PrimalStatus()) == MOI.FeasiblePoint

            @test MOI.canget(instance, MOI.ObjectiveValue())
            @test MOI.get(instance, MOI.ObjectiveValue()) ≈ -2 atol=atol rtol=rtol

            @test MOI.canget(instance, MOI.VariablePrimal(), MOI.VariableIndex)
            @test MOI.get(instance, MOI.VariablePrimal(), x) ≈ 1 atol=atol rtol=rtol
            @test MOI.get(instance, MOI.VariablePrimal(), y) ≈ 1 atol=atol rtol=rtol
            @test MOI.get(instance, MOI.VariablePrimal(), z) ≈ 0 atol=atol rtol=rtol
        end

    end
end

function intsoctests(instance::MOI.AbstractInstance; atol=Base.rtoldefault(Float64), rtol=Base.rtoldefault(Float64))
    intsoc1test(instance, atol=atol, rtol=rtol)
end

function intconictests(instance::MOI.AbstractInstance; atol=Base.rtoldefault(Float64), rtol=Base.rtoldefault(Float64))
    intsoctests(instance, atol=atol, rtol=rtol)
end
