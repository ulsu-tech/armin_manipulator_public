#include <chained_ik.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>

InverseSolverFromKDL::InverseSolverFromKDL()
{
    ourChain.addSegment( KDL::Segment( std::string("world -> "),
            KDL::Joint( std::string("world_joint0"), KDL::Joint::None),
            KDL::Frame( KDL::Rotation::RotX(3.14), {0, 0, 0.})));
    ourChain.addSegment( KDL::Segment( std::string("world -> base_link"),
            KDL::Joint( std::string("world_joint"), KDL::Joint::None),
            KDL::Frame( KDL::Rotation::RotX(1.5708), {0, 0, -0.407})));

    ourChain.addSegment( KDL::Segment( std::string("base_link tip -> a1_link tip"),
            KDL::Joint( std::string("a1_joint"), KDL::Joint::RotY),
            KDL::Frame( KDL::Rotation::RotX(3.1416), {0.2175, -0.118, 0})));
    ourChain.addSegment( KDL::Segment( std::string("a1_link tip -> a2_link tip"),
            KDL::Joint( std::string("a2_joint"), KDL::Joint::RotX, -1),
            KDL::Frame( KDL::Rotation::RotZ(3.1416), {-0.0935, 0.5, 0})));
    ourChain.addSegment( KDL::Segment( std::string("a2_link tip -> a3_link tip"),
            KDL::Joint( std::string("a3_joint"), KDL::Joint::RotX, -1),
            KDL::Frame( KDL::Rotation::RotZ(3.1416), {0.065001,-0.1505, 0})));
    ourChain.addSegment( KDL::Segment( std::string("a3_link  tip -> a4_link tip"),
            KDL::Joint( std::string("a4_joint"), KDL::Joint::RotY, -1),
            KDL::Frame( KDL::Rotation::Identity(), {0.1055, 0.3765, 0})));

    ourChain.addSegment( KDL::Segment( std::string("a4_link tip -> a5_link tip"),
            KDL::Joint( std::string("a5_joint"), KDL::Joint::RotX, 1),
            KDL::Frame( KDL::Rotation::Identity(), {0.0175, 0.174, 0})));

    ourChain.addSegment( KDL::Segment( std::string("a5_link tip -> a6_link tip"),
            KDL::Joint( std::string("a6_joint"), KDL::Joint::RotY, -1),
            KDL::Frame( KDL::Rotation::RotX(M_PI), {0, 0.24537, 0})));
    
    jointsCount = ourChain.getNrOfJoints();

    fksolverO = new KDL::ChainFkSolverPos_recursive(ourChain);
    iksolver1v = new KDL::ChainIkSolverVel_pinv(ourChain);//Inverse velocity solver
    iksolver2v = new KDL::ChainIkSolverVel_wdls(ourChain);
    iksolver3v = new KDL::ChainIkSolverVel_wdls(ourChain);

    Eigen::MatrixXd Mx = Eigen::MatrixXd::Identity(6,6);
    // setting only last 3 coordinates to become valuable
        Mx(0,0) = 0.2; Mx(1,1) = 0.2; Mx(2,2) = 0.2;
        Mx(3,3) = 1; Mx(4,4)= 1; Mx(5,5) = 1;
    iksolver2v->setWeightTS(Mx);
        Mx(0,0) = 1.; Mx(1,1) = 1.; Mx(2,2) = 1.;
        Mx(3,3) = 0.0; Mx(4,4) = 0.0; Mx(5,5) = 0.0;
    iksolver3v->setWeightTS(Mx);
}

InverseSolverFromKDL::~InverseSolverFromKDL()
{
    delete fksolverO;
    delete iksolver1v;
    delete iksolver2v;
}

bool InverseSolverFromKDL::convertJointsToFrame(KDL::Frame &correspondingFrame, const KDL::JntArray &jointPositions)
{
    return fksolverO->JntToCart(jointPositions, correspondingFrame) >=0;
}

bool InverseSolverFromKDL::getJointForFrameStartingJoints(KDL::JntArray & resultedJointPosition, const KDL::Frame & desiredFramePosition, const KDL::JntArray &startingJointPosition, int maxIterations, double accuracy)
{
    KDL::ChainIkSolverPos_NR iksolver1(ourChain,*fksolverO,*iksolver1v, maxIterations, accuracy);
    return iksolver1.CartToJnt(startingJointPosition, desiredFramePosition,
                resultedJointPosition) >= 0;
}

bool InverseSolverFromKDL::getJointForFrameStartingJointsDismissOrientation(KDL::JntArray & resultedJointPosition, const KDL::Frame & desiredFramePosition, const KDL::JntArray &startingJointPosition, int maxIterations, double accuracy)
{
    KDL::ChainIkSolverPos_NR iksolver(ourChain,*fksolverO,*iksolver3v, maxIterations, accuracy);
    return iksolver.CartToJnt(startingJointPosition, desiredFramePosition,
                resultedJointPosition) >= 0;
}


bool InverseSolverFromKDL::getJointForRotationStartingJoints(KDL::JntArray &result, const KDL::Frame &desirOrientation, const KDL::JntArray &startPos, int maxIter, double accuracy)
{
    KDL::ChainIkSolverPos_NR iksolver1(ourChain,*fksolverO,*iksolver2v, maxIter, accuracy);
    auto r = iksolver1.CartToJnt(startPos, desirOrientation, result);
    return r >= 0;
}

bool InverseSolverFromKDL::getLimitedJointForRotationStartingJoints(KDL::JntArray &result, const KDL::Frame &desirOrientation, const KDL::JntArray &startPos, int maxIter, double accuracy)
{
    KDL::JntArray qMin(ourChain.getNrOfJoints()), qMax(ourChain.getNrOfJoints());
    qMin(0) = -2.*M_PI; qMax(0)= 2.* M_PI;
    qMin(1) = -2.33; qMax(1) = 2.33;
    qMin(2) = -M_PI; qMax(2) = M_PI;
    qMin(3) = -M_PI; qMax(3) = M_PI;
    qMin(4) = -M_PI; qMax(4) = M_PI;
    qMin(5) = -M_PI; qMax(5) = M_PI;


    KDL::ChainIkSolverPos_NR_JL iksolver1(ourChain, qMin, qMax, *fksolverO,*iksolver2v, maxIter, accuracy);
    auto r = iksolver1.CartToJnt(startPos, desirOrientation, result);
    return r >= 0;
}

int ChainIkSolverPos_NR_wr::CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out)
{
// copy of
//    int ChainIkSolverPos_NR::CartToJnt(const JntArray& q_init, const Frame& p_in, JntArray& q_out)
//    {
            q_out = q_init;

            unsigned int i;
            for(i=0;i<maxiter;i++){
                fksolver.JntToCart(q_out,f);
                delta_twist = diff(f,p_in);
                const int rc = iksolver.CartToJnt(q_out,delta_twist,delta_q);
                if (E_NOERROR > rc)
                    return (error = E_IKSOLVER_FAILED);
                // we chose to continue if the child solver returned a positive
                // "error", which may simply indicate a degraded solution
                Add(q_out,delta_q,q_out);
                if(Equal(delta_twist,KDL::Twist::Zero(),eps))
                    // converged, but possibly with a degraded solution
                    return (rc > E_NOERROR ? E_DEGRADED : E_NOERROR);
            }
            return (error = E_NO_CONVERGE);        // failed to converge
//    }
}

