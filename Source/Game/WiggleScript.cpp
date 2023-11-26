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
	int index = ChainTail;
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
	deltaLengths.Resize(targetNodes.Count());
	radialVelocities.Resize(targetNodes.Count());
	std::fill(angles.begin(), angles.end(), Vector3::Zero);
	std::fill(angularVelocities.begin(), angularVelocities.end(), Vector3::Zero);
	std::fill(deltaLengths.begin(), deltaLengths.end(), 0);
	std::fill(radialVelocities.begin(), radialVelocities.end(), 0);

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

	// All `local` in the following code means actor-space local
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
		deltaLengths[i] += radialVelocities[i] * deltaTime;

		// Prepare the rotation axis
		auto axisParent = Vector3::Cross(angles[i], normalVectors[i]);
		axisParent.Normalize();

		// Calculate its should-be position in parent space
		auto intendedTransLocal = offsetMatrices[i];
		// Apply rotation
		intendedTransLocal = Matrix::RotationAxis(axisParent, angles[i].Length()) * intendedTransLocal;
		// Apply translation (before rotation)
		intendedTransLocal = Matrix::Translation(-normalVectors[i] * deltaLengths[i]) * intendedTransLocal;

		// Convert from parent space to actor-local space
		mats[targetNodes[i]] = intendedTransLocal * prevNodeTrans;
		prevNodeTrans = mats[targetNodes[i]];
	}

	// mats[TargetNode] = prevMatrix * Matrix::Translation(velocity * deltaTime);
	model->SetCurrentPose(mats);
}

/// <summary>
/// Calculate physics in parent space
/// </summary>
/// <param name="gravity"></param>
/// <param name="index"></param>
/// <param name="delta"></param>
void WiggleScript::SolvePhysics(const Vector3& gravity, int index, float delta)
{
	// Inertial force
	auto acceleration = -AccelerationFactor * hangPointAcc;
	// Gravity
	// acceleration += gravity;

	auto radialAcc = Vector3::Project(acceleration, normalVectors[index]);
	auto angularAcc = acceleration - radialAcc;
	auto radialAccScalar = radialAcc.Length();

	// Angular restore
	angularAcc -= AngularStiffness * angles[index];
	// Angular damping
	angularAcc -= AngularDamping * angularVelocities[index];
	angularVelocities[index] += angularAcc * delta;

	// Radial Restore
	radialAccScalar -= Stiffness * deltaLengths[index];
	// Radial Damping
	radialAccScalar -= Damping * radialVelocities[index];
	radialVelocities[index] += radialAccScalar * delta;
}

void WiggleScript::UpdateAcceleration(const Vector3& hangPointPos, const Matrix& worldToLocal, float delta) {
	auto v = (hangPointPos - hangPointPrevPos) / delta;
	hangPointPrevPos = hangPointPos;
	hangPointAcc = (v - hangPointPrevVel) / delta;
	hangPointPrevVel = v;

	Vector3::TransformNormal(hangPointAcc, worldToLocal, hangPointAcc);
}
