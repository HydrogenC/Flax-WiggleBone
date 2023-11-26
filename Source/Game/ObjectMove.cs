using System;
using System.Collections.Generic;
using FlaxEngine;

namespace Game;

/// <summary>
/// ObjectMove Script.
/// </summary>
public class ObjectMove : Script
{
    public float MoveSpeed
    {
        get;
        set;
    } = 1000;

    AnimatedModel model;

    /// <inheritdoc/>
    public override void OnStart()
    {
        // Here you can add code that needs to be called when script is created, just before the first game update
        model = Actor as AnimatedModel;
    }

    /// <inheritdoc/>
    public override void OnEnable()
    {
        // Here you can add code that needs to be called when script is enabled (eg. register for events)
    }

    /// <inheritdoc/>
    public override void OnDisable()
    {
        // Here you can add code that needs to be called when script is disabled (eg. unregister from events)
    }

    /// <inheritdoc/>
    public override void OnUpdate()
    {
        // Here you can add code that needs to be called every frame
        Vector3 speed = Vector3.Zero;
        if (Input.GetKey(KeyboardKeys.U))
        {
            speed += new Vector3(1, 0, 0);
        }
        if (Input.GetKey(KeyboardKeys.J))
        {
            speed += new Vector3(-1, 0, 0);
        }
        if (Input.GetKey(KeyboardKeys.H))
        {
            speed += new Vector3(0, 0, 1);
        }
        if (Input.GetKey(KeyboardKeys.K))
        {
            speed += new Vector3(0, 0, -1);
        }
        if (Input.GetKey(KeyboardKeys.Y))
        {
            speed += new Vector3(0, 1, 0);
        }
        if (Input.GetKey(KeyboardKeys.I))
        {
            speed += new Vector3(0, -1, 0);
        }
        speed.Normalize();
        speed *= MoveSpeed;

        var trans = model.Transform;
        trans.Translation += speed * Time.DeltaTime;
        model.Transform = trans;
    }
}
