#pragma once

#include <string>
#include <algorithm>
#include <Engine/Scripting/Script.h>
#include <Engine/Engine/Time.h>
#include <Engine/Level/Actors/AnimatedModel.h>
#include <Engine/Level/Actor.h>
#include <Engine/Debug/DebugLog.h>
#include <Engine/Physics/Physics.h>
#include <Engine/Input/Input.h>
#include <Engine/Input/Keyboard.h>

API_CLASS() class GAME_API WiggleScript : public Script
{
	API_AUTO_SERIALIZATION();
	DECLARE_SCRIPTING_TYPE(WiggleScript);

	API_FIELD() float Stiffness = 11;
	API_FIELD() float AccelerationFactor = 0.005;
	API_FIELD() float Damping = 1;
	API_FIELD() int HangNode = -1;
	API_FIELD() Array<int> TargetNodes;

	// [Script]
	void OnEnable() override;
	void OnDisable() override;
	void OnUpdate() override;

private:
	AnimatedModel* model;
	float totalTime = 0;

	// Offset matrix of target from hang node in hang node space
	Array<Matrix> offsetMatrices;
	// normal vector in hang node space
	Array<Vector3> normalVectors;

	Vector3 hangPointPrevPos;
	Vector3 hangPointPrevVel = Vector3::Zero;
	Vector3 hangPointAcc = Vector3::Zero;

	// Length is angle, direction is direction
	Array<Vector3> angularVelocities;
	Array <Vector3> angles;

	void ProcessInput(float delta);
	void SolvePhysics(const Vector3& gravity, int index, float delta);
	void UpdateAcceleration(const Vector3& hangPointPos, const Matrix& worldToLocal, float delta);
};
