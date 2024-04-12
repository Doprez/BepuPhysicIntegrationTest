﻿using System.Diagnostics;
using BepuPhysics;
using BepuPhysics.Constraints;
using Stride.BepuPhysics.Constraints;
using Stride.BepuPhysics.Constraints.CustomConstraints;

namespace Stride.BepuPhysics.Systems;

internal sealed class ConstraintData<T> : ConstraintDataBase where T : unmanaged, IConstraintDescription<T>
{
    private readonly ConstraintComponent<T> _constraintComponent;
    private readonly BepuConfiguration _bepuConfig;
    private ConstraintHandle _cHandle = new(-1);
    private BepuSimulation? _bepuSimulation;
    private bool _exist = false;

    public override bool Exist => _exist;

    public ConstraintData(ConstraintComponent<T> constraintComponent, BepuConfiguration bepuConfig)
    {
        _constraintComponent = constraintComponent;
        _bepuConfig = bepuConfig;
    }

    internal override void RebuildConstraint()
    {
        DestroyConstraint();

        if (!_constraintComponent.Enabled)
            return;

        foreach (var container in _constraintComponent.Bodies)
        {
            if (container is null || container.BodyReference.HasValue == false)
                return; // need to wait for a body to be attached or instanced
        }

        var simIndex = _constraintComponent.Bodies[0]!.SimulationIndex;
        Span<BodyHandle> bodies = stackalloc BodyHandle[_constraintComponent.Bodies.Length];
        int count = 0;

        _bepuSimulation = _bepuConfig.BepuSimulations[simIndex];

        foreach (var component in _constraintComponent.Bodies)
        {
            Debug.Assert(component is not null);
            Debug.Assert(component.BodyReference.HasValue);

#warning maybe send a warning, like the missing camera notification in the engine, instead of exception at runtime
            if (ReferenceEquals(component.Simulation, _bepuSimulation) == false)
                throw new Exception("A constraint between object with different SimulationIndex is not possible");

            bodies[count++] = component.BodyReference.Value.Handle;
        }

        Span<BodyHandle> validBodies = bodies[..count];

		_cHandle = _bepuSimulation.Simulation.Solver.Add(validBodies, _constraintComponent.BepuConstraint);
        _exist = true;
    }

    internal override void DestroyConstraint()
    {
        if (_cHandle.Value != -1 && _bepuSimulation != null && _bepuSimulation.Simulation.Solver.ConstraintExists(_cHandle))
        {
            _bepuSimulation.Simulation.Solver.Remove(_cHandle);
            _cHandle = new(-1);
        }

        _bepuSimulation = null;
        _exist = false;
    }

    internal override void TryUpdateDescription()
    {
        if (_bepuSimulation != null && _cHandle.Value != -1 && _bepuSimulation.Simulation.Solver.ConstraintExists(_cHandle))
        {
            _bepuSimulation.Simulation.Solver.ApplyDescription(_cHandle, _constraintComponent.BepuConstraint);
        }
    }
}