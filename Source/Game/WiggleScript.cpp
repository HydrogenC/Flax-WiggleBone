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
	int index=ChainTail;
	while (index != ChainHead && index >= 0) {
		targetNodes.Add(index);
		index = nodes[index].ParentIndex;
	}

	// Reverse the order to make it from head to tail
	targetNodes.Reverse();

	Array<Matrix> mats;
	model->GetCurrentPose(mats);
	Matrix worldMat;
	model->GetLocalToWorldMatrix(worldMat);
	hangPointPrevPos = (mats[ChainHead] * worldMat).GetTranslation();

	offsetMatrices.Resize(targetNodes.Count());
	normalVectors.Resize(targetNodes.Count());

	angles.Resize(targetNodes.Count());
	angularVelocities.Resize(targetNodes.Count());
	std::fill(angles.begin(), angles.end(), Vector3::Zero);
	std::fill(angularVelocities.begin(), angularVelocities.end(), Vector3::Zero);

	// For convertion from actor space to parent space
	Matrix invPrevNode = Matrix::Invert(mats[ChainHead]);
	Vector3 prevNodePos = mats[ChainHead].GetTranslation();

	for (int i = 0; i < targetNodes.Count(); i++)
	{
		auto currentNodePos = mats[targetNodes[i]].GetTranslation();
		auto posOffset = prevNodePos - currentNodePos;
		Vector3::TransformNormal(posOffset, invPrevNode, normalVectors[i]);
		normalVectors[i].Normalize();
		offsetMatrices[i] = mats[targetNodes[i]] * invPrevNode;

		prevNodePos = currentNodePos;
		invPrevNode = Matrix::Invert(mats[targetNodes[i]]);
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

	Array<Matrix> mats;
	model->GetCurrentPose(mats);

	// Prepare world-local matrices
	Matrix localToWorld;
	model->GetLocalToWorldMatrix(localToWorld);
	Matrix worldToLocal;
	Matrix::Invert(localToWorld, worldToLocal);

	// Resolve acceleration
	Vector3 hangPointPosWorld;
	Vector3::Transform(mats[ChainHead].GetTranslation(), localToWorld, hangPointPosWorld);
	UpdateAcceleration(hangPointPosWorld, worldToLocal, deltaTime);

	// Resolve gravity
	Vector3 localGravity;
	Vector3::TransformNormal(Physics::GetGravity(), worldToLocal, localGravity);
	localGravity *= GravityFactor;

	Matrix prevNodeTrans = mats[ChainHead];
	for (int i = 0; i < targetNodes.Count(); i++)
	{

		SolvePhysics(localGravity, i, deltaTime);
		angles[i] = Float3::ClampLength(angles[i] + angularVelocities[i] * deltaTime, PI / 3);

		// Do parent space to local space conversion
		Vector3 normalVectorLocal, angleLocal;
		Vector3::TransformNormal(normalVectors[i], prevNodeTrans, normalVectorLocal);
		Vector3::TransformNormal(angles[i], prevNodeTrans, angleLocal);
		auto axis = Vector3::Cross(angles[i], normalVectorLocal);
		// Prepare the rotation axis
		axis.Normalize();

		// Calculate its should-be position
		auto intendedTransLocal = offsetMatrices[i] * prevNodeTrans;
		// Apply the rotation from its should-be position
		mats[targetNodes[i]] = Matrix::RotationAxis(axis, angles[i].Length()) * intendedTransLocal;
		prevNodeTrans = mats[targetNodes[i]];
	}

	// mats[TargetNode] = prevMatrix * Matrix::Translation(velocity * deltaTime);
	model->SetCurrentPose(mats);
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
	auto radialAcc = Vector3::Project(acceleration, normalVectors[index]);
	auto angularAcc = acceleration - radialAcc;
	angularAcc -= Damping * angularVelocities[index];
	angularVelocities[index] += angularAcc * delta;
}

void WiggleScript::UpdateAcceleration(const Vector3& hangPointPos, const Matrix& worldToLocal, float delta) {
	auto v = (hangPointPos - hangPointPrevPos) / delta;
	hangPointPrevPos = hangPointPos;
	hangPointAcc = (v - hangPointPrevVel) / delta;
	hangPointPrevVel = v;

	Vector3::TransformNormal(hangPointAcc, worldToLocal, hangPointAcc);
}
