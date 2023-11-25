#include "WiggleScript.h"

WiggleScript::WiggleScript(const SpawnParams& params)
	: Script(params)
{
	// Enable ticking OnUpdate function
	_tickUpdate = true;
}

void WiggleScript::OnEnable()
{
	// Here you can add code that needs to be called when script is enabled (eg. register for events)
	model = (AnimatedModel*)GetActor();
	auto& nodes = model->SkinnedModel->Skeleton.Nodes;
	if (HangNode < 0) {
		HangNode = nodes[TargetNodes[0]].ParentIndex;
	}

	Array<Matrix> mats;
	model->GetCurrentPose(mats);
	Matrix worldMat;
	model->GetLocalToWorldMatrix(worldMat);
	hangPointPrevPos = (mats[HangNode] * worldMat).GetTranslation();

	offsetMatrices.Resize(TargetNodes.Count());
	normalVectors.Resize(TargetNodes.Count());

	angles.Resize(TargetNodes.Count());
	angularVelocities.Resize(TargetNodes.Count());
	std::fill(angles.begin(), angles.end(), Vector3::Zero);
	std::fill(angularVelocities.begin(), angularVelocities.end(), Vector3::Zero);

	// For convertion from actor space to parent space
	Matrix invPrevNode = Matrix::Invert(mats[HangNode]);
	Vector3 prevNodePos = mats[HangNode].GetTranslation();

	for (int i=0;i<TargetNodes.Count();i++)
	{
		auto currentNodePos = mats[TargetNodes[i]].GetTranslation();
		auto posOffset = prevNodePos - currentNodePos;
		posOffset.Normalize();
		Vector3::TransformNormal(posOffset, invPrevNode, normalVectors[i]);
		offsetMatrices[i] = mats[TargetNodes[i]] * invPrevNode;

		prevNodePos = currentNodePos;
		invPrevNode = Matrix::Invert(mats[TargetNodes[i]]);
	}
}

void WiggleScript::OnDisable()
{
	// Here you can add code that needs to be called when script is disabled (eg. unregister from events)
}

void WiggleScript::OnUpdate()
{
	// Here you can add code that needs to be called every frame
	float deltaTime = Time::GetDeltaTime();
	totalTime += deltaTime;

	Array<Matrix> mats;
	model->GetCurrentPose(mats);

	Matrix localToWorld;
	model->GetLocalToWorldMatrix(localToWorld);
	Matrix worldToLocal;
	Matrix::Invert(localToWorld, worldToLocal);

	Vector3 hangPointPosWorld;
	Vector3::Transform(mats[HangNode].GetTranslation(), localToWorld, hangPointPosWorld);
	UpdateAcceleration(hangPointPosWorld, worldToLocal, deltaTime);

	Vector3 localGravity;
	Vector3::TransformNormal(Physics::GetGravity(), worldToLocal, localGravity);

	Matrix prevNodeTrans= mats[HangNode];
	for (int i = 0; i < TargetNodes.Count(); i++)
	{
		auto intendedTrans = offsetMatrices[i] * prevNodeTrans;
		SolvePhysics(0.01 * localGravity, i, deltaTime);

		angles[i] = Float3::ClampLength(angles[i] + angularVelocities[i] * deltaTime, PI / 3);
		auto axis = Vector3::Cross(angles[i], normalVectors[i]);
		axis.Normalize();
		mats[TargetNodes[i]] = Matrix::RotationAxis(axis, angles[i].Length()) * intendedTrans;
		prevNodeTrans = mats[TargetNodes[i]];
	}

	// mats[TargetNode] = prevMatrix * Matrix::Translation(velocity * deltaTime);
	model->SetCurrentPose(mats);
	ProcessInput(deltaTime);
}

void WiggleScript::SolvePhysics(const Vector3& gravity, int index, float delta)
{
	// F = kx
	auto acceleration = -Stiffness * angles[index];
	acceleration -= AccelerationFactor * hangPointAcc;
	// Gravity
	acceleration += gravity;
	// Damping
	// acceleration -= Damping * velocity;

	// TODO: Add length spring
	auto angularAcc = Vector3::ProjectOnPlane(acceleration, normalVectors[index]);
	angularAcc -= Damping * angularVelocities[index];
	angularVelocities[index] += angularAcc * delta;
	// velocity += acceleration * delta;
}

void WiggleScript::ProcessInput(float delta)
{
	Vector3 speed = Vector3::Zero;
	if (Input::GetKey(KeyboardKeys::U)) {
		speed += Vector3(1, 0, 0);
	}
	if (Input::GetKey(KeyboardKeys::J)) {
		speed += Vector3(-1, 0, 0);
	}
	if (Input::GetKey(KeyboardKeys::H)) {
		speed += Vector3(0, 0, 1);
	}
	if (Input::GetKey(KeyboardKeys::K)) {
		speed += Vector3(0, 0, -1);
	}
	speed.Normalize();
	speed *= 1000;

	auto trans = model->GetTransform();
	trans.Translation += speed * delta;
	model->SetTransform(trans);
}

void WiggleScript::UpdateAcceleration(const Vector3& hangPointPos, const Matrix& worldToLocal, float delta) {
	auto v = (hangPointPos - hangPointPrevPos) / delta;
	hangPointPrevPos = hangPointPos;
	hangPointAcc = (v - hangPointPrevVel) / delta;
	hangPointPrevVel = v;

	Vector3::TransformNormal(hangPointAcc, worldToLocal, hangPointAcc);
}
