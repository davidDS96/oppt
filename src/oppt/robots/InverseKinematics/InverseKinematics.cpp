#include "oppt/robotHeaders/InverseKinematics/InverseKinematics.hpp"
#include <urdf_parser/urdf_parser.h>

using namespace oppt;

TracIKSolver::TracIKSolver(RandomGenerator* randGen,
                           const std::string &urdfFile,
                           const std::string &baseLink,
                           const std::string &tipLink):
	IKSolver(randGen, urdfFile, baseLink, tipLink) {
}

bool TracIKSolver::init() {	
	urdf::ModelInterfaceSharedPtr robotModel = urdf::parseURDFFile(urdfFile_);
	cout << "base link: " << baseLink_ << endl;
	cout << "tipLink: " << tipLink_ << endl;
	/////////////////////////////
	auto links = robotModel->links_;
	for (auto &link : links) {
		cout << "link: " << link.first << endl;
	}
	/////////////////////////////
	kdlTree_ = std::make_unique<KDL::Tree>();
	if (!(kdl_parser::treeFromUrdfModel(*(robotModel.get()), *(kdlTree_.get())))) {
		WARNING("KDL Tree could not be initialized");
		return false;
	}
	chain_ = std::make_unique<KDL::Chain>();
	if (!(kdlTree_->getChain(baseLink_, tipLink_, *(chain_.get())))) {
		WARNING("KDL chain could not be initialized with base link '" + baseLink_ + "'' and tip link '" + tipLink_ + "'");
		return false;
	}

	////////////////////////////////
	for (auto &segment: chain_->segments) {
		cout << "segment: " << segment.getName() << endl;
		auto trans = segment.getFrameToTip().p;
		auto rot = segment.getFrameToTip().M;
		FloatType r, p, y;
		rot.GetRPY(r, p ,y);
		cout << "(" << trans[0] << ", " << trans[1] << ", " << trans[2] << ", " << r << ", " << p << ", " << y << ")" << endl;

	}
	////////////////////////////////

	jacobianSolver_ = std::make_unique<KDL::ChainJntToJacSolver>(*(chain_.get()));
	jacobian_ = std::make_unique<KDL::Jacobian>(chain_->getNrOfJoints());
	q_.resize(chain_->getNrOfJoints());


	///////////////////////////
	for (auto &segment : chain_->segments) {
		cout << "joint: " << segment.getJoint().getName() << endl;
	}
	///////////////////////////

	lb.resize(chain_->getNrOfJoints());
	ub.resize(chain_->getNrOfJoints());

	std::vector<KDL::Segment> chainSegs = chain_->segments;
	urdf::JointConstSharedPtr joint;
	uint jointNum = 0;
	for (unsigned int i = 0; i != chainSegs.size(); ++i) {
		joint = robotModel->getJoint(chainSegs[i].getJoint().getName());
		if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
			jointNum++;
			float lower, upper;
			int hasLimits;
			if ( joint->type != urdf::Joint::CONTINUOUS ) {
				if (joint->safety) {
					if (inFixedJoints(chainSegs[i].getJoint().getName())) {
						auto jv = fixedJoints_.at(chainSegs[i].getJoint().getName());
						lower = jv;
						upper = jv;
					} else {
						lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
						upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
					}
				} else {
					if (inFixedJoints(chainSegs[i].getJoint().getName())) {
						auto jv = fixedJoints_.at(chainSegs[i].getJoint().getName());
						lower = jv;
						upper = jv;
					} else {
						lower = joint->limits->lower;
						upper = joint->limits->upper;
					}
				}
				hasLimits = 1;
			}
			else {
				hasLimits = 0;
			}
			if (hasLimits) {
				lb(jointNum - 1) = lower;
				ub(jointNum - 1) = upper;
			}
			else {
				lb(jointNum - 1) = std::numeric_limits<float>::lowest();
				ub(jointNum - 1) = std::numeric_limits<float>::max();
			}
		}
	}

	FloatType timeout = 0.05;
	FloatType eps = 1e-5;

	ikSolver_ = std::make_unique<TRAC_IK::TRAC_IK>(*(chain_.get()), lb, ub, timeout, eps);
	initialized_ = true;
	return true;

	//TRAC_IK::TRAC_IK ikSolver(chain, );

}

std::pair<bool, VectorFloat> TracIKSolver::solve(const VectorFloat &pose,
        const unsigned int &numAttempts,
        const VectorFloat &qInit) {
	if (!initialized_)
		return std::make_pair(false, VectorFloat());
	KDL::Vector translation(pose[0], pose[1], pose[2]);
	auto rotation = KDL::Rotation::Quaternion(pose[6], pose[3], pose[4], pose[5]);
	KDL::Frame kdlPose(rotation, translation);
	unsigned int numJoints = chain_->getNrOfJoints();
	KDL::JntArray qInit_, result;
	qInit_.resize(numJoints);
	for (size_t i = 0; i != numAttempts; ++i) {
		if (qInit.size() == numJoints) {
			for (size_t j = 0; j != numJoints; ++j) {				
				qInit_(i) = qInit[i];
			}
		} else {
			for (size_t j = 0; j != numJoints; ++j) {
				qInit_(j) = sampleRandomAngle(lb(j), ub(j));
			}
		}

		int resultCode = ikSolver_->CartToJnt(qInit_, kdlPose, result);
		if (resultCode < 0) {
			// Unsuccessful
			continue;
		}

		VectorFloat resultingJointAngles(numJoints, 0.0);
		for (size_t j = 0; j != numJoints; ++j) {
			resultingJointAngles[j] = result(j);
		}

		return std::make_pair(true, resultingJointAngles);
	}

	return std::make_pair(false, VectorFloat());
}

FloatType TracIKSolver::sampleRandomAngle(const FloatType &lower, const FloatType &upper) {
	std::uniform_real_distribution<> dis(lower, upper);
	return dis(*(randGen_));
}

std::unique_ptr<IKSolver> TracIKSolver::clone() const {
	std::unique_ptr<IKSolver> clonedIkSolver(new TracIKSolver(randGen_, urdfFile_, baseLink_, tipLink_));
	return std::move(clonedIkSolver);
}

void TracIKSolver::setFixedJoints(std::map<std::string, FloatType> &fixedJoints) {
	fixedJoints_ = fixedJoints;
}

bool TracIKSolver::inFixedJoints(const std::string &jointName) const {
	for (auto &entry : fixedJoints_) {
		if (entry.first.find(jointName) != std::string::npos) {
			return true;
		}
	}

	return false;
}

VectorFloat TracIKSolver::inverseJacobian(const VectorFloat &q, const VectorFloat &x1, const VectorFloat &x2) const {
	KDL::ChainIkSolverVel_pinv inverseJ(*(chain_.get()));
	VectorFloat deltaX = subtractVectors(x2, x1);	
	for (size_t i = 0; i != q.size(); ++i) {
		q_(i) = q[i];
	}

	KDL::Twist twist;	
	twist.vel = KDL::Vector(deltaX[0], deltaX[1], deltaX[2]);
	twist.rot = KDL::Vector(deltaX[3], deltaX[4], deltaX[5]);
	printVector(deltaX, "deltaX");
	KDL::JntArray qDot;
	qDot.resize(q.size());
	inverseJ.CartToJnt(q_, twist, qDot);

	VectorFloat qDotVec(q.size());
	for (size_t i = 0; i != qDotVec.size(); ++i) {
		qDotVec[i] = qDot(i);
	}
	
	return addVectors(q, qDotVec);
    /**auto qEigen = toEigenVec(q);
    auto x1Eigen = toEigenVec(x1);
    auto x2Eigen = toEigenVec(x2);

    auto deltaX = x2Eigen - x1Eigen;

    cout << "q: " << qEigen << endl;
    cout << "x1Eigen: " << x1Eigen << endl;
    cout << "x2Eigen: " << x2Eigen << endl;
    cout << "deltaX: " << deltaX << endl;

	for (size_t i = 0; i != q.size(); ++i) {
		q_(i) = q[i];
	}

	jacobianSolver_->JntToJac(q_, *(jacobian_.get()));

	// Put the jacobian into an eigen matrix
	Matrixdf jacobianMatrix = Matrixdf::Zero(6, chain_->getNrOfJoints());
	for (size_t i = 0; i != 6; ++i) {
		for (size_t j = 0; j != chain_->getNrOfJoints(); ++j) {
			jacobianMatrix(i, j) = (*(jacobian_.get()))(i, j);
		}
	}

	cout << "jacobianMatrix: " << endl << jacobianMatrix << endl;
	Matrixdf jTranspose = jacobianMatrix.transpose();

	Vectordf dotTheta = jTranspose * (jacobianMatrix * jTranspose).inverse() * deltaX;

	//Vectordf dotTheta = jacobianMatrix.colPivHouseholderQr().solve(deltaX);

    // Calculate the pseudo inverse of the jacobian matrix using orthogonal decomposition
    // and hope for the best
	//Matrixdf pInv = jacobianMatrix.completeOrthogonalDecomposition().pseudoInverse();
	//cout << "pInv: " << endl << pInv << endl;
	//auto dotTheta = pInv * deltaX;
	cout << "dotTheta: " << dotTheta << endl;
	Vectordf qNew = qEigen + dotTheta;

	cout << "qNew: " << qNew << endl;
	getchar();
	return toStdVec<FloatType>(qNew);*/
}