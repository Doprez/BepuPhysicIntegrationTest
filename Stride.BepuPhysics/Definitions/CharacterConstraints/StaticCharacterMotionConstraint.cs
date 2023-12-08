﻿using System.Numerics;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;

namespace Stride.BepuPhysics.Definitions.CharacterConstraints;
//IMPORTANT NOTE: The static and dynamic motion constraint variants were autogenerated from a text template. To modify the constraint, modify the text template instead.
//Using code generators to generate these sorts of constraint variants can work around some of the limitations in compile time expressiveness-
//here, we avoid creating a bunch of duplicate math for the one and two body constraint cases which would have to be separately maintained.
//There are other cases where this is pushed even further. Within the engine, the dozens of contact constraint variants are generated by text templates.
//This isn't the most wonderful solution in terms of development experience or tooling, 
//but it maximizes performance and avoids the need for in-language turing complete metaprogramming, so I'm okay with it.

//Constraint descriptions provide an explicit mapping from the array-of-structures format to the internal array-of-structures-of-arrays format used by the solver.
//Note that there is a separate description for the one and two body case- constraint implementations take advantage of the lack of a second body to reduce data gathering requirements.
/// <summary>
/// Description of a character motion constraint where the support is static.
/// </summary>
public struct StaticCharacterMotionConstraint : IOneBodyConstraintDescription<StaticCharacterMotionConstraint>
{
    /// <summary>
    /// Maximum force that the horizontal motion constraint can apply to reach the current velocity goal.
    /// </summary>
    public float MaximumHorizontalForce;
    /// <summary>
    /// Maximum force that the vertical motion constraint can apply to fight separation.
    /// </summary>
    public float MaximumVerticalForce;
    /// <summary>
    /// Target horizontal velocity in terms of the basis X and -Z axes.
    /// </summary>
    public Vector2 TargetVelocity;
    /// <summary>
    /// Depth of the supporting contact. The vertical motion constraint permits separating velocity if, after a frame, the objects will still be touching.
    /// </summary>
    public float Depth;
    /// <summary>
    /// Stores the quaternion-packed orthonormal basis for the motion constraint. When expanded into a matrix, X and Z will represent the Right and Backward directions respectively. Y will represent Up.
    /// In other words, a target tangential velocity of (4, 2) will result in a goal velocity of 4 along the (1, 0, 0) * Basis direction and a goal velocity of 2 along the (0, 0, -1) * Basis direction.
    /// All motion moving along the (0, 1, 0) * Basis axis will be fought against by the vertical motion constraint.
    /// </summary>
    public Quaternion SurfaceBasis;
    /// <summary>
    /// World space offset from the character's center to apply impulses at.
    /// </summary>
    public Vector3 OffsetFromCharacterToSupportPoint;

    //It's possible to create multiple descriptions for the same underlying constraint type id which can update different parts of the constraint data.
    //This functionality isn't used very often, though- you'll notice that the engine has a 1:1 mapping (at least at the time of this writing).
    //But in principle, it doesn't have to be that way. So, the description must provide information about the type and type id.
    /// <summary>
    /// Gets the constraint type id that this description is associated with. 
    /// </summary>
    public static int ConstraintTypeId => StaticCharacterMotionTypeProcessor.BatchTypeId;

    /// <summary>
    /// Gets the TypeProcessor type that is associated with this description.
    /// </summary>
    public static Type TypeProcessorType => typeof(StaticCharacterMotionTypeProcessor);
    /// <summary>
    /// Creates a type processor for this constraint type.
    /// </summary>
    public static TypeProcessor CreateTypeProcessor() => new StaticCharacterMotionTypeProcessor();

    //Note that these mapping functions use a "GetOffsetInstance" function. Each CharacterMotionPrestep is a bundle of multiple constraints;
    //by grabbing an offset instance, we're selecting a specific slot in the bundle to modify. For simplicity and to guarantee consistency of field strides,
    //we refer to that slot using the same struct and then write only to the first slot.
    //(Note that accessing slots after the first may result in access violations; the 'offset instance' is not guaranteed to refer to valid data beyond the first slot!)
    public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
    {
        ref var target = ref GatherScatter.GetOffsetInstance(ref Buffer<StaticCharacterMotionPrestep>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
        QuaternionWide.WriteFirst(SurfaceBasis, ref target.SurfaceBasis);
        GatherScatter.GetFirst(ref target.MaximumHorizontalForce) = MaximumHorizontalForce;
        GatherScatter.GetFirst(ref target.MaximumVerticalForce) = MaximumVerticalForce;
        Vector2Wide.WriteFirst(TargetVelocity, ref target.TargetVelocity);
        GatherScatter.GetFirst(ref target.Depth) = Depth;
        Vector3Wide.WriteFirst(OffsetFromCharacterToSupportPoint, ref target.OffsetFromCharacter);
    }

    public static void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out StaticCharacterMotionConstraint description)
    {
        ref var source = ref GatherScatter.GetOffsetInstance(ref Buffer<StaticCharacterMotionPrestep>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
        QuaternionWide.ReadFirst(source.SurfaceBasis, out description.SurfaceBasis);
        description.MaximumHorizontalForce = GatherScatter.GetFirst(ref source.MaximumHorizontalForce);
        description.MaximumVerticalForce = GatherScatter.GetFirst(ref source.MaximumVerticalForce);
        Vector2Wide.ReadFirst(source.TargetVelocity, out description.TargetVelocity);
        description.Depth = GatherScatter.GetFirst(ref source.Depth);
        Vector3Wide.ReadFirst(source.OffsetFromCharacter, out description.OffsetFromCharacterToSupportPoint);
    }
}