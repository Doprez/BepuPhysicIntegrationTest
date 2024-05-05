﻿using Stride.BepuPhysics.Definitions;
using Stride.BepuPhysics.Navigation.Components;
using Stride.Core;
using Stride.Core.Mathematics;
using Stride.Core.Threading;
using Stride.Engine;
using Stride.Games;
using System.Collections.Concurrent;

namespace Stride.BepuPhysics.Navigation.Processors;
public class RecastPhysicsNavigationProcessor : EntityProcessor<RecastPhysicsNavigationComponent>
{
	private RecastMeshProcessor? _recastMeshProcessor;
	private List<RecastPhysicsNavigationComponent> _components = new();
	public readonly ConcurrentQueue<RecastPhysicsNavigationComponent> TryGetPathQueue = new();

	public RecastPhysicsNavigationProcessor()
	{
		//run after the RecastMeshProcessor
		Order = 20001;
	}

	protected override void OnSystemAdd()
	{
		ServicesHelper.LoadBepuServices(Services);
		_recastMeshProcessor = Services.GetService<RecastMeshProcessor>();
		if (_recastMeshProcessor is null)
		{
			// add the RecastMeshProcessor if it doesn't exist
			_recastMeshProcessor = new RecastMeshProcessor(Services);
			// add to the Scenes processors
			var sceneSystem = Services.GetSafeServiceAs<SceneSystem>();
			sceneSystem.Game.GameSystems.Add(_recastMeshProcessor);
		}
	}

	protected override void OnEntityComponentAdding(Entity entity, RecastPhysicsNavigationComponent component, RecastPhysicsNavigationComponent data)
	{
		_components.Add(component);
	}

	protected override void OnEntityComponentRemoved(Entity entity, RecastPhysicsNavigationComponent component, RecastPhysicsNavigationComponent data)
	{
		_components.Remove(component);
	}

	public override void Update(GameTime time)
	{
		var deltaTime = (float)time.Elapsed.TotalSeconds;

		for (int i = 0; i < 10; i++)
		{
			if (TryGetPathQueue.IsEmpty) break;

			if (TryGetPathQueue.TryDequeue(out var pathfinding))
			{
				// cannot use dispatcher here because of the TryFindPath method.
				SetNewPath(pathfinding);
			}
		}

		Dispatcher.For(0, _components.Count, i =>
		{
			if (_components[i].ShouldMove)
			{
				Move(_components[i], deltaTime);
				Rotate(_components[i]);
			}

			if (_components[i].SetNewPath && !_components[i].InSetPathQueue)
			{
				TryGetPathQueue.Enqueue(_components[i]);
				_components[i].InSetPathQueue = true;
				_components[i].SetNewPath = false;
			}
		});
	}

	private void SetNewPath(RecastPhysicsNavigationComponent pathfinder)
	{
		pathfinder.InSetPathQueue = false;
		if (_recastMeshProcessor.TryFindPath(pathfinder.Entity.Transform.WorldMatrix.TranslationVector, pathfinder.Target, ref pathfinder.Polys, ref pathfinder.Path))
		{
			pathfinder.SetNewPath = false;
		}
	}

	private void Move(RecastPhysicsNavigationComponent pathfinder, float deltaTime)
	{
		if (pathfinder.Path.Count == 0)
		{
			pathfinder.SetNewPath = true;
			return;
		}

		var position = pathfinder.Entity.Transform.WorldMatrix.TranslationVector;

		var nextWaypointPosition = pathfinder.Path[0];
		var distanceToWaypoint = Vector3.Distance(position, nextWaypointPosition);

		// When the distance between the character and the next waypoint is large enough, move closer to the waypoint
		if (distanceToWaypoint > 0.5)
		{
			var direction = nextWaypointPosition - position;
			direction.Normalize();
			//direction *= pathfinder.Speed;

			pathfinder.PhysicsComponent.Move(direction);
		}
		else
		{
			if (pathfinder.Path.Count > 0)
			{
				// need to test if storing the index in Pathfinder would be faster than this.
				pathfinder.Path.RemoveAt(0);
			}
		}
	}

	public void Rotate(RecastPhysicsNavigationComponent pathfinder)
	{
		if (pathfinder.Path.Count == 0)
		{
			return;
		}
		var position = pathfinder.Entity.Transform.WorldMatrix.TranslationVector;

		float angle = (float)Math.Atan2(pathfinder.Path[0].Z - position.Z,
			pathfinder.Path[0].X - position.X);

		pathfinder.Entity.Transform.Rotation = Quaternion.RotationY(-angle);
		pathfinder.PhysicsComponent.Orientation = pathfinder.Entity.Transform.Rotation;
	}
}
