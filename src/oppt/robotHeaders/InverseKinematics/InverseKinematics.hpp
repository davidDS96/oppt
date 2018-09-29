#ifndef __INVERSE_KINEMATICS_HPP__
#define __INVERSE_KINEMATICS_HPP__
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <kdl/jntarray.hpp>
#include <trac_ik/trac_ik.hpp>
#include "IKSolver.hpp"

namespace oppt {
class TracIKSolver: public IKSolver {
public:
	TracIKSolver(RandomGenerator* randGen,
	             const std::string &urdfFile,
	             const std::string &baseLink,
	             const std::string &tipLink);

	virtual ~TracIKSolver() {}

	virtual std::pair<bool, VectorFloat> solve(const VectorFloat &pose,
	        const unsigned int &numAttempts,
	        const VectorFloat &qInit = VectorFloat()) override;

	virtual std::unique_ptr<IKSolver> clone() const override;

	bool init();

	void setFixedJoints(std::map<std::string, FloatType> &fixedJoints);

	VectorFloat inverseJacobian(const VectorFloat &q, const VectorFloat &x1, const VectorFloat &x2) const;

private:
	std::unique_ptr<KDL::Tree> kdlTree_ = nullptr;

	std::unique_ptr<KDL::Chain> chain_ = nullptr;

	std::unique_ptr<TRAC_IK::TRAC_IK> ikSolver_ = nullptr;

	std::unique_ptr<KDL::ChainJntToJacSolver> jacobianSolver_ = nullptr;

	mutable std::unique_ptr<KDL::Jacobian> jacobian_ = nullptr;	

	KDL::JntArray lb, ub;
	mutable KDL::JntArray q_;

	bool initialized_ = false;

	std::map<std::string, FloatType> fixedJoints_;

private:
	FloatType sampleRandomAngle(const FloatType &lower, const FloatType &upper);

	bool inFixedJoints(const std::string &jointName) const;	

};
}

#endif