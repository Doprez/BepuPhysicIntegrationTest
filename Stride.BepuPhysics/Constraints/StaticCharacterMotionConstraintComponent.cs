using Stride.BepuPhysics.Constraints.CustomConstraints;
using Stride.BepuPhysics.Systems;
using Stride.Engine.Design;
using Stride.Engine;
using Stride.Core;
using Stride.Core.Mathematics;

namespace Stride.BepuPhysics.Constraints;
[DataContract]
[DefaultEntityComponentProcessor(typeof(ConstraintProcessor), ExecutionMode = ExecutionMode.Runtime)]
[ComponentCategory("Bepu - Constraint")]
public class StaticCharacterMotionConstraintComponent : OneBodyConstraintComponent<StaticCharacterMotionConstraint>
{
	public StaticCharacterMotionConstraintComponent() => BepuConstraint = new();

	/// <summary>
	/// Maximum force that the horizontal motion constraint can apply to reach the current velocity goal.
	/// </summary>
	public float MaximumHorizontalForce
	{
		get => BepuConstraint.MaximumHorizontalForce;
		set
		{
			BepuConstraint.MaximumHorizontalForce = value;
			ConstraintData?.TryUpdateDescription();
		}
	}
	/// <summary>
	/// Maximum force that the vertical motion constraint can apply to fight separation.
	/// </summary>
	public float MaximumVerticalForce
	{
		get => BepuConstraint.MaximumVerticalForce;
		set
		{
			BepuConstraint.MaximumVerticalForce = value;
			ConstraintData?.TryUpdateDescription();
		}
	}
	/// <summary>
	/// Target horizontal velocity in terms of the basis X and -Z axes.
	/// </summary>
	public Vector2 TargetVelocity
	{
		get => BepuConstraint.TargetVelocity.ToStrideVector();
		set
		{
			BepuConstraint.TargetVelocity = value.ToNumericVector();
			ConstraintData?.TryUpdateDescription();
		}
	}

	/// <summary>
	/// Depth of the supporting contact. The vertical motion constraint permits separating velocity if, after a frame, the objects will still be touching.
	/// </summary>
	public float Depth
	{
		get => BepuConstraint.Depth;
		set
		{
			BepuConstraint.Depth = value;
			ConstraintData?.TryUpdateDescription();
		}
	}
	/// <summary>
	/// Stores the quaternion-packed orthonormal basis for the motion constraint. When expanded into a matrix, X and Z will represent the Right and Backward directions respectively. Y will represent Up.
	/// In other words, a target tangential velocity of (4, 2) will result in a goal velocity of 4 along the (1, 0, 0) * Basis direction and a goal velocity of 2 along the (0, 0, -1) * Basis direction.
	/// All motion moving along the (0, 1, 0) * Basis axis will be fought against by the vertical motion constraint.
	/// </summary>
	public Quaternion SurfaceBasis
	{
		get => BepuConstraint.SurfaceBasis.ToStrideQuaternion();
		set
		{
			BepuConstraint.SurfaceBasis = value.ToNumericQuaternion();
			ConstraintData?.TryUpdateDescription();
		}
	}
	/// <summary>
	/// World space offset from the character's center to apply impulses at.
	/// </summary>
	public Vector3 OffsetFromCharacterToSupportPoint
	{
		get => BepuConstraint.OffsetFromCharacterToSupportPoint.ToStrideVector();
		set
		{
			BepuConstraint.OffsetFromCharacterToSupportPoint = value.ToNumericVector();
			ConstraintData?.TryUpdateDescription();
		}
	}

	internal override ConstraintDataBase CreateProcessorData(BepuConfiguration bepuConfiguration)
	{
		bepuConfiguration.BepuSimulations[A.SimulationIndex].Simulation.Solver.Register<StaticCharacterMotionConstraint>();

		return base.CreateProcessorData(bepuConfiguration);
	}
}
