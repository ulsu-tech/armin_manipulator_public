#ifndef CHAINED_IK_H
#define CHAINED_IK_H
#include <config.h>

#ifdef orocos_kdl_FOUND
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

class ChainIkSolverPos_NR_wr : public KDL::ChainIkSolverPos
{
        const KDL::Chain chain;
        KDL::ChainIkSolverVel& iksolver;
        KDL::ChainFkSolverPos& fksolver;
        KDL::JntArray delta_q;
        KDL::Frame f;
        KDL::Twist delta_twist;

        unsigned int maxiter;
        double eps;
  public:
        static const int E_IKSOLVER_FAILED = -100; //! Child IK solver failed
    ChainIkSolverPos_NR_wr(const KDL::Chain& _chain,
        KDL::ChainFkSolverPos& _fksolver, KDL::ChainIkSolverVel& _iksolver,
        unsigned int _maxiter=100, double _eps=1e-6) :
chain(_chain),iksolver(_iksolver),fksolver(_fksolver),delta_q(_chain.getNrOfJoints()),
        maxiter(_maxiter),eps(_eps) {};
    virtual int CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out);

};

class InverseSolverFromKDL
{
    KDL::Chain ourChain;
    unsigned int jointsCount;
    KDL::ChainFkSolverPos_recursive *fksolverO;
    KDL::ChainIkSolverVel_pinv *iksolver1v;
    KDL::ChainIkSolverVel_wdls *iksolver2v;
    KDL::ChainIkSolverVel_wdls *iksolver3v;
  public:
    InverseSolverFromKDL();
    ~InverseSolverFromKDL();
    bool convertJointsToFrame(KDL::Frame &,const KDL::JntArray &);
    bool getJointForFrameStartingJoints(KDL::JntArray &, const KDL::Frame &,const KDL::JntArray &, int = 100, double = 1e-6);
    bool getJointForFrameStartingJointsDismissOrientation(KDL::JntArray &, const KDL::Frame &,const KDL::JntArray &, int = 100, double = 1e-6);
    bool getJointForRotationStartingJoints(KDL::JntArray &, const KDL::Frame &,const KDL::JntArray &, int = 100, double = 1e-6);
    bool getLimitedJointForRotationStartingJoints(KDL::JntArray &, const KDL::Frame &,const KDL::JntArray &, int = 100, double = 1e-6);
    int getJointsCount() const { return jointsCount;};
};
#endif

#endif //CHAINED_IK_H

