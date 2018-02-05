# TODO: Move generic instance tests from MOIU to here

struct UnknownSet <: MOI.AbstractSet end

function nametest(instance::MOI.AbstractInstance)
    @testset "Name test" begin
        @test MOI.get(instance, MOI.NumberOfVariables()) == 0
        @test MOI.get(instance, MOI.NumberOfConstraints{MOI.ScalarAffineFunction{Float64},MOI.LessThan{Float64}}()) == 0

        v = MOI.addvariables!(instance, 2)
        @test MOI.canget(instance, MOI.VariableName(), typeof(v[1]))
        @test MOI.get(instance, MOI.VariableName(), v[1]) == ""

        @test MOI.canset(instance, MOI.VariableName(), typeof(v[1]))
        MOI.set!(instance, MOI.VariableName(), v[1], "")
        MOI.set!(instance, MOI.VariableName(), v[2], "") # Shouldn't error with duplicate empty name

        MOI.set!(instance, MOI.VariableName(), v[1], "Var1")
        @test_throws Exception MOI.set!(instance, MOI.VariableName(), v[2], "Var1")
        MOI.set!(instance, MOI.VariableName(), v[2], "Var2")

        @test MOI.canget(instance, MOI.VariableIndex, "Var1")
        @test !MOI.canget(instance, MOI.VariableIndex, "Var3")

        @test MOI.get(instance, MOI.VariableIndex, "Var1") == v[1]
        @test MOI.get(instance, MOI.VariableIndex, "Var2") == v[2]
        @test_throws KeyError MOI.get(instance, MOI.VariableIndex, "Var3")

        MOI.set!(instance, MOI.VariableName(), v, ["VarX","Var2"])
        @test MOI.get(instance, MOI.VariableName(), v) == ["VarX", "Var2"]

        if MOI.candelete(instance, v[2])
            MOI.delete!(instance, v[2])
            @test !MOI.canget(instance, MOI.VariableIndex, "Var2")
            @test_throws KeyError MOI.get(instance, MOI.VariableIndex, "Var2")
        end

        c = MOI.addconstraint!(instance, MOI.ScalarAffineFunction(v, [1.0,1.0], 0.0), MOI.LessThan(1.0))
        c2 = MOI.addconstraint!(instance, MOI.ScalarAffineFunction(v, [-1.0,1.0], 0.0), MOI.EqualTo(0.0))
        @test MOI.canget(instance, MOI.ConstraintName(), typeof(c))
        @test MOI.get(instance, MOI.ConstraintName(), c) == ""

        @test MOI.canset(instance, MOI.ConstraintName(), typeof(c))
        MOI.set!(instance, MOI.ConstraintName(), c, "")
        MOI.set!(instance, MOI.ConstraintName(), c2, "") # Shouldn't error with duplicate empty name

        MOI.set!(instance, MOI.ConstraintName(), c, "Con0")
        @test MOI.get(instance, MOI.ConstraintName(), c) == "Con0"
        @test_throws Exception MOI.set!(instance, MOI.ConstraintName(), c2, "Con0")

        MOI.set!(instance, MOI.ConstraintName(), [c], ["Con1"])
        @test MOI.get(instance, MOI.ConstraintName(), [c]) == ["Con1"]


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

function emptytest(instance::MOI.AbstractInstance)
    # Taken from LIN1
    v = MOI.addvariables!(instance, 3)
    vc = MOI.addconstraint!(instance, MOI.VectorOfVariables(v), MOI.Nonnegatives(3))
    c = MOI.addconstraint!(instance, MOI.VectorAffineFunction([1,1,1,2,2], [v;v[2];v[3]], ones(5), [-3.0,-2.0]), MOI.Zeros(2))
    MOI.set!(instance, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(), MOI.ScalarAffineFunction(v, [-3.0, -2.0, -4.0], 0.0))
    MOI.set!(instance, MOI.ObjectiveSense(), MOI.MinSense)

    @test !MOI.isempty(instance)

    MOI.empty!(instance)

    @test MOI.isempty(instance)

    @test MOI.get(instance, MOI.NumberOfVariables()) == 0
    @test MOI.get(instance, MOI.NumberOfConstraints{MOI.VectorOfVariables,MOI.Nonnegatives}()) == 0
    @test MOI.get(instance, MOI.NumberOfConstraints{MOI.VectorAffineFunction{Float64},MOI.Zeros}()) == 0
    @test isempty(MOI.get(instance, MOI.ListOfConstraints()))

    @test !MOI.isvalid(instance, v[1])
    @test !MOI.isvalid(instance, vc)
    @test !MOI.isvalid(instance, c)
end

abstract type BadInstance <: MOI.AbstractInstance end
MOI.get(src::BadInstance, ::MOI.ListOfInstanceAttributesSet) = MOI.AbstractInstanceAttribute[]
MOI.get(src::BadInstance, ::MOI.ListOfVariableIndices) = [MOI.VariableIndex(1)]
MOI.get(src::BadInstance, ::MOI.ListOfVariableAttributesSet) = MOI.AbstractVariableAttribute[]
MOI.get(src::BadInstance, ::MOI.ListOfConstraints) = [(MOI.SingleVariable, MOI.EqualTo{Float64})]
MOI.get(src::BadInstance, ::MOI.ListOfConstraintIndices{F,S}) where {F,S} = [MOI.ConstraintIndex{F,S}(1)]
MOI.get(src::BadInstance, ::MOI.ConstraintFunction, ::MOI.ConstraintIndex{MOI.SingleVariable,MOI.EqualTo{Float64}}) = MOI.SingleVariable(MOI.VariableIndex(1))
MOI.get(src::BadInstance, ::MOI.ConstraintSet, ::MOI.ConstraintIndex{MOI.SingleVariable,MOI.EqualTo{Float64}}) = MOI.EqualTo(0.0)
MOI.get(src::BadInstance, ::MOI.ListOfConstraintAttributesSet) = MOI.AbstractConstraintAttribute[]

struct BadConstraintInstance <: BadInstance end
MOI.get(src::BadConstraintInstance, ::MOI.ListOfConstraints) = [(MOI.SingleVariable, MOI.EqualTo{Float64}), (MOI.SingleVariable, UnknownSet)]
MOI.get(src::BadInstance, ::MOI.ConstraintFunction, ::MOI.ConstraintIndex{MOI.SingleVariable,UnknownSet}) = MOI.SingleVariable(MOI.VariableIndex(1))
MOI.get(src::BadInstance, ::MOI.ConstraintSet, ::MOI.ConstraintIndex{MOI.SingleVariable,UnknownSet}) = UnknownSet()

struct BadInstanceAttribute <: MOI.AbstractInstanceAttribute end
struct BadInstanceAttributeInstance <: BadInstance end
MOI.canget(src::BadInstanceAttributeInstance, ::BadInstanceAttribute) = true
MOI.get(src::BadInstanceAttributeInstance, ::BadInstanceAttribute) = 0
MOI.get(src::BadInstanceAttributeInstance, ::MOI.ListOfInstanceAttributesSet) = MOI.AbstractInstanceAttribute[BadInstanceAttribute()]

struct BadVariableAttribute <: MOI.AbstractVariableAttribute end
struct BadVariableAttributeInstance <: BadInstance end
MOI.canget(src::BadVariableAttributeInstance, ::BadVariableAttribute, ::Type{MOI.VariableIndex}) = true
MOI.get(src::BadVariableAttributeInstance, ::BadVariableAttribute, ::Vector{MOI.VariableIndex}) = [0]
MOI.get(src::BadVariableAttributeInstance, ::MOI.ListOfVariableAttributesSet) = MOI.AbstractVariableAttribute[BadVariableAttribute()]

struct BadConstraintAttribute <: MOI.AbstractConstraintAttribute end
struct BadConstraintAttributeInstance <: BadInstance end
MOI.canget(src::BadConstraintAttributeInstance, ::BadConstraintAttribute, ::Type{<:MOI.ConstraintIndex}) = true
MOI.get(src::BadConstraintAttributeInstance, ::BadConstraintAttribute, ::Vector{<:MOI.ConstraintIndex}) = [0]
MOI.get(src::BadConstraintAttributeInstance, ::MOI.ListOfConstraintAttributesSet) = MOI.AbstractConstraintAttribute[BadConstraintAttribute()]

function failcopytest(dest::MOI.AbstractInstance, src::MOI.AbstractInstance, expected_status)
    copyresult = MOI.copy!(dest, src)
    @test copyresult.status == expected_status
end

failcopytestc(dest::MOI.AbstractInstance) = failcopytest(dest, BadConstraintInstance(), MOI.CopyUnsupportedConstraint)
failcopytestia(dest::MOI.AbstractInstance) = failcopytest(dest, BadInstanceAttributeInstance(), MOI.CopyUnsupportedAttribute)
failcopytestva(dest::MOI.AbstractInstance) = failcopytest(dest, BadVariableAttributeInstance(), MOI.CopyUnsupportedAttribute)
failcopytestca(dest::MOI.AbstractInstance) = failcopytest(dest, BadConstraintAttributeInstance(), MOI.CopyUnsupportedAttribute)

function copytest(dest::MOI.AbstractInstance, src::MOI.AbstractInstance)
    v = MOI.addvariables!(src, 3)
    csv = MOI.addconstraint!(src, MOI.SingleVariable(v[2]), MOI.EqualTo(2.))
    cvv = MOI.addconstraint!(src, MOI.VectorOfVariables(v), MOI.Nonnegatives(3))
    csa = MOI.addconstraint!(src, MOI.ScalarAffineFunction([v[3], v[1]], [1., 3.], 2.), MOI.LessThan(2.))
    cva = MOI.addconstraint!(src, MOI.VectorAffineFunction([1, 2], [v[3], v[2]], ones(2), [-3.0,-2.0]), MOI.Zeros(2))
    MOI.set!(src, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(), MOI.ScalarAffineFunction(v, [-3.0, -2.0, -4.0], 0.0))
    MOI.set!(src, MOI.ObjectiveSense(), MOI.MinSense)

    copyresult = MOI.copy!(dest, src)
    @test copyresult.status == MOI.CopySuccess
    dict = copyresult.indexmap

    @test MOI.get(dest, MOI.NumberOfVariables()) == 3
    @test MOI.get(dest, MOI.NumberOfConstraints{MOI.SingleVariable,MOI.EqualTo{Float64}}()) == 1
    @test MOI.get(dest, MOI.ListOfConstraintIndices{MOI.SingleVariable,MOI.EqualTo{Float64}}()) == [dict[csv]]
    @test MOI.get(dest, MOI.NumberOfConstraints{MOI.VectorOfVariables,MOI.Nonnegatives}()) == 1
    @test MOI.get(dest, MOI.ListOfConstraintIndices{MOI.VectorOfVariables,MOI.Nonnegatives}()) == [dict[cvv]]
    @test MOI.get(dest, MOI.NumberOfConstraints{MOI.ScalarAffineFunction{Float64},MOI.LessThan{Float64}}()) == 1
    @test MOI.get(dest, MOI.ListOfConstraintIndices{MOI.ScalarAffineFunction{Float64},MOI.LessThan{Float64}}()) == [dict[csa]]
    @test MOI.get(dest, MOI.NumberOfConstraints{MOI.VectorAffineFunction{Float64},MOI.Zeros}()) == 1
    @test MOI.get(dest, MOI.ListOfConstraintIndices{MOI.VectorAffineFunction{Float64},MOI.Zeros}()) == [dict[cva]]
    loc = MOI.get(dest, MOI.ListOfConstraints())
    @test length(loc) == 4
    @test (MOI.SingleVariable,MOI.EqualTo{Float64}) in loc
    @test (MOI.VectorOfVariables,MOI.Nonnegatives) in loc
    @test (MOI.ScalarAffineFunction{Float64},MOI.LessThan{Float64}) in loc
    @test (MOI.VectorAffineFunction{Float64},MOI.Zeros) in loc

    @test MOI.canget(dest, MOI.ConstraintFunction(), typeof(dict[csv]))
    @test MOI.get(dest, MOI.ConstraintFunction(), dict[csv]) == MOI.SingleVariable(dict[v[2]])
    @test MOI.canget(dest, MOI.ConstraintSet(), typeof(dict[csv]))
    @test MOI.get(dest, MOI.ConstraintSet(), dict[csv]) == MOI.EqualTo(2.)
    @test MOI.canget(dest, MOI.ConstraintFunction(), typeof(dict[cvv]))
    @test MOI.get(dest, MOI.ConstraintFunction(), dict[cvv]) == MOI.VectorOfVariables(getindex.(dict, v))
    @test MOI.canget(dest, MOI.ConstraintSet(), typeof(dict[cvv]))
    @test MOI.get(dest, MOI.ConstraintSet(), dict[cvv]) == MOI.Nonnegatives(3)
    @test MOI.canget(dest, MOI.ConstraintFunction(), typeof(dict[csa]))
    @test MOI.get(dest, MOI.ConstraintFunction(), dict[csa]) ≈ MOI.ScalarAffineFunction([dict[v[3]], dict[v[1]]], [1., 3.], 2.)
    @test MOI.canget(dest, MOI.ConstraintSet(), typeof(dict[csa]))
    @test MOI.get(dest, MOI.ConstraintSet(), dict[csa]) == MOI.LessThan(2.)
    @test MOI.canget(dest, MOI.ConstraintFunction(), typeof(dict[cva]))
    @test MOI.get(dest, MOI.ConstraintFunction(), dict[cva]) ≈ MOI.VectorAffineFunction([1, 2], [dict[v[3]], dict[v[2]]], ones(2), [-3.0,-2.0])
    @test MOI.canget(dest, MOI.ConstraintSet(), typeof(dict[cva]))
    @test MOI.get(dest, MOI.ConstraintSet(), dict[cva]) == MOI.Zeros(2)

    @test MOI.canget(dest, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}())
    @test MOI.get(dest, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}()) ≈ MOI.ScalarAffineFunction([dict[v[1]], dict[v[2]], dict[v[3]]], [-3.0, -2.0, -4.0], 0.0)
    @test MOI.canget(dest, MOI.ObjectiveSense())
    @test MOI.get(dest, MOI.ObjectiveSense()) == MOI.MinSense
end

function canaddconstrainttest(instance::MOI.AbstractInstance, ::Type{GoodT}, ::Type{BadT}) where {GoodT, BadT}
    v = MOI.addvariable!(instance)
    @test MOI.canaddconstraint(instance, MOI.SingleVariable, MOI.EqualTo{GoodT})
    @test MOI.canaddconstraint(instance, MOI.ScalarAffineFunction{GoodT}, MOI.EqualTo{GoodT})
    # Bad type
    @test !MOI.canaddconstraint(instance, MOI.ScalarAffineFunction{BadT}, MOI.EqualTo{GoodT})
    @test !MOI.canaddconstraint(instance, MOI.ScalarAffineFunction{BadT}, MOI.EqualTo{BadT})
    @test !MOI.canaddconstraint(instance, MOI.SingleVariable, MOI.EqualTo{BadT})

    @test MOI.canaddconstraint(instance, MOI.VectorOfVariables, MOI.Zeros)
    @test !MOI.canaddconstraint(instance, MOI.VectorOfVariables, MOI.EqualTo{GoodT}) # vector in scalar
    @test !MOI.canaddconstraint(instance, MOI.SingleVariable, MOI.Zeros) # scalar in vector
    @test !MOI.canaddconstraint(instance, MOI.VectorOfVariables, UnknownSet) # set not supported
end
