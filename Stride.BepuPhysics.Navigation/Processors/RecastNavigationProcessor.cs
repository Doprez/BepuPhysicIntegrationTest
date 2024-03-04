using DotRecast.Core.Numerics;
using DotRecast.Detour;
using Stride.BepuPhysics.Definitions;
using Stride.BepuPhysics.Navigation.Components;
using Stride.BepuPhysics.Navigation.Extensions;
using Stride.Core;
using Stride.Core.Mathematics;
using Stride.Core.Threading;
using Stride.Engine;
using Stride.Games;
using Stride.Navigation;
using Stride.Profiling;
using System.Diagnostics;
using System.Xml.Linq;

namespace Stride.BepuPhysics.Navigation.Processors;
public class RecastNavigationProcessor : EntityProcessor<RecastNavigationComponent>
{
	private RecastMeshProcessor? _recastMeshProcessor;
	private List<RecastNavigationComponent> _components = new();

	private Stopwatch _stopwatch = new();
	private DebugTextSystem _debugTextSystem;

	public RecastNavigationProcessor()
	{
		//run after the RecastMeshProcessor
		Order = 20001;
	}

	protected override void OnSystemAdd()
	{
		ServicesHelper.LoadBepuServices(Services);
		_recastMeshProcessor = Services.GetService<RecastMeshProcessor>();
		if(_recastMeshProcessor is null)
		{
			// add the RecastMeshProcessor if it doesn't exist
			_recastMeshProcessor = new RecastMeshProcessor(Services);
			// add to the Scenes processors
			var sceneSystem = Services.GetSafeServiceAs<SceneSystem>();
			sceneSystem.Game.GameSystems.Add(_recastMeshProcessor);
		}
		_debugTextSystem = Services.GetService<DebugTextSystem>();
	}

	override protected void OnEntityComponentAdding(Entity entity, RecastNavigationComponent component, RecastNavigationComponent data)
	{
		_components.Add(component);
	}

	override protected void OnEntityComponentRemoved(Entity entity, RecastNavigationComponent component, RecastNavigationComponent data)
	{
		_components.Remove(component);
	}

	public override void Update(GameTime time)
	{
		var deltaTime = (float)time.Elapsed.TotalSeconds;

		//_stopwatch.Start();

		for (int i = 0; i < _components.Count; i++)
		{
			if (_components[i].SetNewPath)
			{
				// cannot use dispatcher here because of the TryFindPath method.
				SetNewPath(_components[i]);
			}
		}

		//_stopwatch.Stop();
		//_debugTextSystem.Print("PathfindingProcessor.SetNewPath: " + _stopwatch.Elapsed.TotalMilliseconds, new Int2(100, 100));
		//
		//_stopwatch.Reset();
		//_stopwatch.Start();

		Dispatcher.For(0, _components.Count, i =>
		{
			//if (_components[i].SetNewPath)
			//{
			//	// cannot use dispatcher here because of the TryFindPath method.
			//	SetNewPath(_components[i]);
			//}
			if (_components[i].ShouldMove)
			{
				Move(_components[i], deltaTime);
				Rotate(_components[i]);
				FindRandomPoint(_components[i]);
			}
		});

		//_stopwatch.Stop();
		//_debugTextSystem.Print("PathfindingProcessor.Move: " + _stopwatch.Elapsed.TotalMilliseconds, new Int2(100, 120));

		//Dispatcher.For(0, _components.Count, i =>
		//{
		//	if (_components[i].ShouldMove)
		//	{
		//		FindRandomPoint(_components[i]);
		//	}
		//});

	}

	private void SetNewPath(RecastNavigationComponent pathfinder)
	{
		if (_recastMeshProcessor.TryFindPath(pathfinder.Entity.Transform.WorldMatrix.TranslationVector, pathfinder.Target, ref pathfinder.Polys, ref pathfinder.Path))
		{
			pathfinder.SetNewPath = false;
		}
	}

	private void Move(RecastNavigationComponent pathfinder, float deltaTime)
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
		if (distanceToWaypoint > 0.1)
		{
			var direction = nextWaypointPosition - position;
			direction.Normalize();
			direction *= pathfinder.Speed * deltaTime;

			position += direction;
		}
		else
		{
			if (pathfinder.Path.Count > 0)
			{
				// need to test if storing the index in Pathfinder would be faster than this.
				pathfinder.Path.RemoveAt(0);
			}
		}

		pathfinder.Entity.Transform.Position = position;
	}

	public void Rotate(RecastNavigationComponent pathfinder)
	{
		if (pathfinder.Path.Count == 0)
		{
			return;
		}
		var position = pathfinder.Entity.Transform.WorldMatrix.TranslationVector;

		float angle = (float)Math.Atan2(pathfinder.Path[0].Z - position.Z,
			pathfinder.Path[0].X - position.X);

		pathfinder.Entity.Transform.Rotation = Quaternion.RotationY(-angle);
	}

	private void FindRandomPoint(RecastNavigationComponent pathfinder)
	{
		if (Vector3.Distance(pathfinder.Entity.Transform.WorldMatrix.TranslationVector, pathfinder.Target) < 1)
		{
			pathfinder.Target.Z = Random.Shared.Next(-200, 200);
			pathfinder.Target.X = Random.Shared.Next(-200, 200);
			pathfinder.SetNewPath = true;
			pathfinder.ShouldMove = true;
		}
	}

}
