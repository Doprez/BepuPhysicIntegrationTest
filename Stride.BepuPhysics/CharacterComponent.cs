﻿using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using Stride.BepuPhysics.Components;
using Stride.BepuPhysics.Definitions;
using Stride.BepuPhysics.Definitions.Contacts;
using Stride.Core;
using Stride.Core.Extensions;
using Stride.Core.Mathematics;
using Stride.Engine;
using NVector3 = System.Numerics.Vector3;
using NRigidPose = BepuPhysics.RigidPose;
using Stride.BepuPhysics.Constraints;

namespace Stride.BepuPhysics;

[ComponentCategory("Bepu")]
public class CharacterComponent : BodyComponent, ISimulationUpdate, IContactEventHandler
{

    public StaticCharacterMotionConstraintComponent MotionConstraint;

    /// <summary>
    /// Movement speed
    /// </summary>
    public float Speed { get; set; } = 10f;
    /// <summary>
    /// Jump force
    /// </summary>
    public float JumpSpeed { get; set; } = 10f;
	public float PeakAirTime { get; set; } = 0.5f;
    public float FallSpeedMultiplier { get; set; } = 2.0f;

	[DataMemberIgnore]
    public Vector3 Velocity { get; set; }
    [DataMemberIgnore]
    public bool IsGrounded { get; private set; }

    /// <summary>
    /// Order is not guaranteed and may change at any moment
    /// </summary>
    [DataMemberIgnore]
    public List<(ContainerComponent Source, Contact Contact)> Contacts { get; } = new();

	private bool _tryJump;
	private bool _didJump;

	private float _airTime;

	public CharacterComponent()
    {
        InterpolationMode = InterpolationMode.Interpolated;
    }

    protected override void AttachInner(NRigidPose containerPose, BodyInertia shapeInertia, TypedIndex shapeIndex)
    {
        base.AttachInner(containerPose, shapeInertia, shapeIndex);

        BodyInertia = new BodyInertia { InverseMass = 1f };
        if (Entity.Get<DebugInfo>() is null)
            Entity.Add(new DebugInfo(this));
        ContactEventHandler = this;
	}

    public void Move(Vector3 direction)
    {
        Velocity = direction * Speed;
    }

    /// <summary>
    /// Will jump if grounded
    /// </summary>
    public void TryJump()
    {
        _tryJump = true;
    }

    public void SimulationUpdate(float simTimeStep)
    {
        Awake = true; // Keep this body active  

        LinearVelocity = new Vector3(Velocity.X, LinearVelocity.Y, Velocity.Z);

		if (_tryJump)
        {
            if (IsGrounded)
            {
                ApplyLinearImpulse(Vector3.UnitY * JumpSpeed);
				_didJump = true;
			}
            _tryJump = false;
        }
    }

    public void AfterSimulationUpdate(float simTimeStep)
	{
		UpdateFallSpeed(simTimeStep);

		CheckGrounded(); // Checking for grounded after simulation ran to compute contacts as soon as possible after they are received
        // If there is no input from the player and we are grounded, ignore gravity to prevent sliding down the slope we might be on
        // Do not ignore if there is any input to ensure we stick to the surface as much as possible while moving down the slope
        IgnoreGlobalGravity = IsGrounded && Velocity.Length() <= 0f;

		if (IsGrounded == true && !_didJump)
		{
			LinearVelocity = new Vector3(0, 0, 0);
		}
        _didJump = false;
	}

    private void CheckGrounded()
    {
        IsGrounded = false;
        if (Simulation == null || Contacts.Count == 0)
            return;

        var gravity = Simulation.PoseGravity.ToNumericVector();
        foreach (var contact in Contacts)
        {
            var contactNormal = contact.Contact.Normal;

            if (NVector3.Dot(gravity, contactNormal) < 0) // If the body is supported by a contact whose surface is against gravity
            {
                IsGrounded = true;
                return;
            }
        }
    }

    private void UpdateFallSpeed(float delta)
	{
		_airTime += delta * FallSpeedMultiplier;
		if (IsGrounded || _airTime <= PeakAirTime)
		{
			_airTime = 0;
		}
		else
		{
			LinearVelocity = new Vector3(0, -9.8f * (_airTime + 1), 0);
		}
	}

    bool IContactEventHandler.NoContactResponse => false;

    void IContactEventHandler.OnStartedTouching<TManifold>(CollidableReference eventSource, CollidableReference other, ref TManifold contactManifold, int contactIndex, BepuSimulation bepuSimulation)
    {
        var otherContainer = bepuSimulation.GetContainer(other);
        contactManifold.GetContact(contactIndex, out var contact);
        contact.Offset = contact.Offset + Entity.Transform.WorldMatrix.TranslationVector.ToNumericVector() + CenterOfMass.ToNumericVector();
        Contacts.Add((otherContainer, contact));
    }

    void IContactEventHandler.OnStoppedTouching<TManifold>(CollidableReference eventSource, CollidableReference other, ref TManifold contactManifold, int contactIndex, BepuSimulation bepuSimulation)
    {
        var otherContainer = bepuSimulation.GetContainer(other);
        for (int i = Contacts.Count - 1; i >= 0; i--)
        {
            if (Contacts[i].Source == otherContainer)
                Contacts.SwapRemoveAt(i);
        }
    }

    class DebugInfo : SyncScript
    {
        readonly CharacterComponent _character;
        public DebugInfo(CharacterComponent character)
        {
            _character = character;
        }

        public override void Update()
        {
            DebugText.Print($"Mouse delta : {Input.MouseDelta}", new Int2(50, 950));
            DebugText.Print($"Velocity : {_character.Velocity}", new Int2(50, 975));
            DebugText.Print($"Orientation : {_character.Orientation}", new Int2(50, 1000));
            DebugText.Print($"IsGrounded : {_character.IsGrounded}", new Int2(50, 1025));
            DebugText.Print($"ContactPoints count : {_character.Contacts.Count}", new Int2(50, 1050));
        }
    }
}


