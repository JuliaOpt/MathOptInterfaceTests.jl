# TODO: Move generic instance tests from MOIU to here

function nametest(instance::MOI.AbstractInstance)
    @testset "Name test" begin
        @test MOI.get(instance, MOI.NumberOfVariables()) == 0
        @test MOI.get(instance, MOI.NumberOfConstraints{MOI.ScalarAffineFunction{Float64},MOI.LessThan{Float64}}()) == 0

        v = MOI.addvariables!(instance, 2)
        @test MOI.canset(instance, MOI.VariableName(), v[1])
        @test MOI.canget(instance, MOI.VariableName(), v[1])
        @test MOI.get(instance, MOI.VariableName(), v[1]) == ""

        MOI.set!(instance, MOI.VariableName(), v[1], "Var1")
        MOI.set!(instance, MOI.VariableName(), v[2], "Var2")

        @test MOI.canget(instance, MOI.VariableIndex, "Var1")
        @test !MOI.canget(instance, MOI.VariableIndex, "Var3")

        @test MOI.get(instance, MOI.VariableIndex, "Var1") == v[1]
        @test MOI.get(instance, MOI.VariableIndex, "Var2") == v[2]
        @test_throws KeyError MOI.get(instance, MOI.VariableIndex, "Var3")

        if MOI.candelete(instance, v[2])
            MOI.delete!(instance, v[2])
            @test !MOI.canget(instance, MOI.VariableIndex, "Var2")
            @test_throws KeyError MOI.get(instance, MOI.VariableIndex, "Var2")
        end

        c = MOI.addconstraint!(instance, MOI.ScalarAffineFunction(v, [1.0,1.0], 0.0), MOI.LessThan(1.0))
        @test MOI.canset(instance, MOI.ConstraintName(), c)
        @test MOI.canget(instance, MOI.ConstraintName(), c)
        @test MOI.get(instance, MOI.ConstraintName(), c) == ""

        MOI.set!(instance, MOI.ConstraintName(), c, "Con1")

        @test MOI.canget(instance, MOI.ConstraintIndex{MOI.ScalarAffineFunction{Float64},MOI.LessThan{Float64}}, "Con1")
        @test !MOI.canget(instance, MOI.ConstraintIndex{MOI.ScalarAffineFunction{Float64},MOI.LessThan{Float64}}, "Con2")
        @test MOI.canget(instance, MOI.ConstraintIndex, "Con1")
        @test !MOI.canget(instance, MOI.ConstraintIndex, "Con2")

        @test MOI.get(instance, MOI.ConstraintIndex{MOI.ScalarAffineFunction{Float64},MOI.LessThan{Float64}}, "Con1") == c
        @test_throws KeyError MOI.get(instance, MOI.ConstraintIndex{MOI.ScalarAffineFunction{Float64},MOI.LessThan{Float64}}, "Con2")
        @test MOI.get(instance, MOI.ConstraintIndex, "Con1") == c
        @test_throws KeyError MOI.get(instance, MOI.ConstraintIndex, "Con2")

        if MOI.candelete(instance, c)
            MOI.delete!(instance, c)
            @test !MOI.canget(instance, MOI.ConstraintIndex{MOI.ScalarAffineFunction{Float64},MOI.LessThan{Float64}}, "Con1")
            @test !MOI.canget(instance, MOI.ConstraintIndex, "Con1")
            @test_throws KeyError MOI.get(instance, MOI.ConstraintIndex{MOI.ScalarAffineFunction{Float64},MOI.LessThan{Float64}}, "Con1")
            @test_throws KeyError MOI.get(instance, MOI.ConstraintIndex, "Con1")
        end

        # TODO: Test for error when duplicate names are assigned
    end
end

# Taken from https://github.com/JuliaOpt/MathOptInterfaceUtilities.jl/issues/41
function validtest(instance::MOI.AbstractInstance)
    v = MOI.addvariables!(instance, 2)
    @test MOI.isvalid(instance, v[1])
    @test MOI.isvalid(instance, v[2])
    x = MOI.addvariable!(instance)
    @test MOI.isvalid(instance, x)
    MOI.delete!(instance, x)
    @test !MOI.isvalid(instance, x)
    cf = MOI.ScalarAffineFunction(v, [1.0,1.0], 0.0)
    c = MOI.addconstraint!(instance, cf, MOI.LessThan(1.0))
    @test MOI.isvalid(instance, c)
    @test !MOI.isvalid(instance, MOI.ConstraintIndex{MOI.ScalarAffineFunction{Float32},MOI.LessThan{Float32}}(1))
    @test !MOI.isvalid(instance, MOI.ConstraintIndex{MOI.ScalarAffineFunction{Float32},MOI.LessThan{Float64}}(1))
    @test !MOI.isvalid(instance, MOI.ConstraintIndex{MOI.ScalarAffineFunction{Float64},MOI.LessThan{Float32}}(1))
    @test !MOI.isvalid(instance, MOI.ConstraintIndex{MOI.VectorQuadraticFunction{Float64},MOI.SecondOrderCone}(1))
end
