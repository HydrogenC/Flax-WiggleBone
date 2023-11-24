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
		HangNode = nodes[TargetNode].ParentIndex;
	}

	Array<Matrix> mats;
	model->GetCurrentPose(mats);
	Matrix worldMat;
	model->GetLocalToWorldMatrix(worldMat);
	hangPointPrevPos = (mats[HangNode] * worldMat).GetTranslation();
	Matrix invHangNode = Matrix::Invert(mats[HangNode]);
	auto posOffset = mats[HangNode].GetTranslation() - mats[TargetNode].GetTranslation();
	posOffset.Normalize();
	Vector3::TransformNormal(posOffset, invHangNode, normalVector);

	offsetMatrix = mats[TargetNode] * invHangNode;
	DebugLog::Log(String::Format(L"Offset matrix: {}", offsetMatrix));
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

	// auto pos = mats[TargetNode].GetTranslation();
	// DebugLog::Log(String::Format(L"Pos: {}", pos));
	auto intendedTrans = offsetMatrix * mats[HangNode];

	Matrix worldToLocal;
	Matrix::Invert(localToWorld, worldToLocal);
	
	Vector3 hangPointPosWorld;
	Vector3::Transform(mats[HangNode].GetTranslation(), localToWorld, hangPointPosWorld);
	DebugLog::Log(String::Format(L"HangPoint Pos World: {}", hangPointPosWorld));
	UpdateAcceleration(hangPointPosWorld, worldToLocal, deltaTime);
	DebugLog::Log(String::Format(L"HangPointAcc: {}", hangPointAcc));

	Vector3 localGravity;
	Vector3::TransformNormal(Physics::GetGravity(), worldToLocal, localGravity);

	SolvePhysics(0.01 * localGravity, deltaTime);
	angle = Float3::ClampLength(angle + angularVelocity * deltaTime, PI / 3);
	auto axis = Vector3::Cross(angle, normalVector);
	axis.Normalize();

	DebugLog::Log(String::Format(L"Angular Velocity: {}", angularVelocity));
	DebugLog::Log(String::Format(L"Angle: {}", angle));
	mats[TargetNode] = Matrix::RotationAxis(axis, angle.Length()) * intendedTrans;

	// mats[TargetNode] = prevMatrix * Matrix::Translation(velocity * deltaTime);

	model->SetCurrentPose(mats);

	ProcessInput(deltaTime);
}

/*
void WiggleScript::LookAtParent(Matrix& trans, const Matrix& parentTrans)
{
	auto upVector = trans.GetUp();
	auto toParentVector = parentTrans.GetTranslation() - trans.GetTranslation();
	auto angle = Vector3::Angle(-upVector, toParentVector);
	auto axis = Vector3::Cross(upVector, toParentVector);
	axis.Normalize();
	trans *= Matrix::RotationAxis(axis, angle);
}
*/

void WiggleScript::SolvePhysics(const Vector3& gravity, float delta)
{
	// F = kx
	auto acceleration = -Stiffness * angle;
	acceleration -= AccelerationFactor * hangPointAcc;
	// Gravity
	acceleration += gravity;
	// Damping
	// acceleration -= Damping * velocity;

	auto angularAcc = Vector3::ProjectOnPlane(acceleration, normalVector);
	angularAcc -= Damping * angularVelocity;
	angularVelocity += angularAcc * delta;
	// velocity += acceleration * delta;
	DebugLog::Log(String::Format(L"Angular Acc: {}", angularAcc));
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
