﻿using DotRecast.Core.Numerics;
using DotRecast.Detour;
using Stride.BepuPhysics.Navigation.Extensions;
using Stride.BepuPhysics.Navigation.Processors;
using Stride.Core;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Engine.Design;

namespace Stride.BepuPhysics.Navigation.Components;
[DataContract(nameof(RecastNavigationComponent))]
[ComponentCategory("Bepu - Navigation")]
[DefaultEntityComponentProcessor(typeof(RecastNavigationProcessor), ExecutionMode = ExecutionMode.Runtime)]
public class RecastNavigationComponent : EntityComponent
{

	public float Speed { get; set; } = 5.0f;

	/// <summary>
	/// True if a new path needs to be calculated, can be manually changed to force a new path to be calculated.
	/// </summary>
	[DataMemberIgnore]
	public bool ShouldMove { get; set; } = true;
	[DataMemberIgnore]
	public bool SetNewPath { get; set; } = true;

	/// <summary>
	/// The target position for the agent to move to. will trigger IsDirty to be set to true.
	/// </summary>
	[DataMemberIgnore]
	public Vector3 Target;

	[DataMemberIgnore]
	public List<Vector3> Path = new();
	[DataMemberIgnore]
	public List<long> Polys = new();
}
