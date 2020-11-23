#include <stdlib.h>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <stdio.h>
#include <iostream>
#include <stdint.h>
#include <string>
#include <sys/time.h>

int main(int argc, char * argv[])
{
    bool kinematics_status;
#if 0
    //Definition of a kinematic chain & add segments to the chain
    KDL::Chain chain;
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(0.0,0.0,1.020))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,0.480))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,0.645))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,0.120))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));


    // Create solver based on kinematic chain
    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
 
    // Create joint array
    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray jointpositions = KDL::JntArray(nj);

    jointpositions(0) = 0.1;
    jointpositions(1) = -0.1;
    jointpositions(2) = 0.1;
    jointpositions(3) = -0.15;
    jointpositions(4) = 0.15;
    jointpositions(5) = 0.2;

    // Create the frame that will contain the results
    KDL::Frame cartpos;


std::cout<<"Solvinf direct task for: "<<std::endl;
    for(auto i = 0; i < nj; ++i)
    {
        std::cout<<"["<<i<<"] = "<<jointpositions(i)<<std::endl;
    }
    // Calculate forward position kinematics
    kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
    if(kinematics_status>=0){
        std::cout << cartpos <<std::endl;
        printf("%s \n","Succes, thanks KDL!");
    }else{
        printf("%s \n","Error: could not calculate forward kinematics :(");
    }

//Creation of the solvers:
KDL::ChainFkSolverPos_recursive fksolver1(chain);//Forward position solver
KDL::ChainIkSolverVel_pinv iksolver1v(chain);//Inverse velocity solver
KDL::ChainIkSolverPos_NR iksolver1(chain,fksolver1,iksolver1v,10000,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6
 
//Creation of jntarrays:
KDL::JntArray q(chain.getNrOfJoints());
KDL::JntArray q_init(chain.getNrOfJoints());
 
    jointpositions(0) = 0.12;
    jointpositions(1) = -0.12;
    jointpositions(2) = 0.12;
    jointpositions(3) = -0.152;
    jointpositions(4) = 0.152;
    jointpositions(5) = 0.22;
//Set destination frame
KDL::Frame F_dest({0.98866,   -0.149988, -0.00746879,
      0.14721,     0.97778,   -0.149251,
    0.0296888,    0.146459,    0.988771}, {-0.00568028,   0.0297705,     2.26125});
 
int ret = iksolver1.CartToJnt(q_init,F_dest,q);
std::cout<<"ret = "<<ret<<std::endl;
    if (! ret) {
        std::cout<<"Solution obtained is:" <<std::endl;
        for(auto i = 0; i < chain.getNrOfJoints(); ++i)
            std::cout<<"\t"<<q_init(i) <<" -> "<<q(i)<<std::endl;
    }

ret = iksolver1.CartToJnt(jointpositions,F_dest,q);
std::cout<<"ret = "<<ret<<std::endl;
    if (! ret) {
        std::cout<<"Solution obtained is:" <<std::endl;
        for(auto i = 0; i < chain.getNrOfJoints(); ++i)
            std::cout<<"\t"<<jointpositions(i) <<" -> "<<q(i)<<std::endl;
    }

#endif

#if 0
    KDL::Chain tryChain;
    tryChain.addSegment( KDL::Segment( KDL::Joint(),
                        KDL::Frame( KDL::Rotation::Identity(), { 1, 0, 0})));
    tryChain.addSegment( KDL::Segment( KDL::Joint(KDL::Joint::RotZ),
                            KDL::Frame( KDL::Rotation::RotX( M_PI/2), { 0, 0, 1} )));
    tryChain.addSegment( KDL::Segment( KDL::Joint(),
                            KDL::Frame( KDL::Rotation::Identity(), { 0, 1, 0} )));


    unsigned int njT = tryChain.getNrOfJoints();
    KDL::JntArray jointpositionsT = KDL::JntArray(njT);

    KDL::Frame cartposT;

std::cout<<"Solving direct task for T: "<<std::endl;
    //jointpositionsO(1) = 0;
    //jointpositionsO(1) = 3.18;
    //jointpositionsO() = -3.14;
    //jointpositionsO(5) = -3.14;
    //jointpositionsT(0) = 1.57;
    for(auto i = 0; i < njT; ++i)
    {
        std::cout<<"["<<i<<"] = "<<jointpositionsT(i)<<std::endl;
    }
    // Calculate forward position kinematics
    KDL::ChainFkSolverPos_recursive fksolverT = KDL::ChainFkSolverPos_recursive(tryChain);
    kinematics_status = fksolverT.JntToCart(jointpositionsT,cartposT);
    if(kinematics_status>=0){
        std::cout << cartposT <<std::endl;
        printf("%s \n","Succes, thanks KDL!");
    }else{
        printf("%s \n","Error: could not calculate forward kinematics :(");
    }
#endif

    // our chain base_link till end-effector
    KDL::Chain ourChain;
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
    
    unsigned int njO = ourChain.getNrOfJoints();
    KDL::JntArray jointpositionsO = KDL::JntArray(njO);
    KDL::JntArray jointpositionsNew = KDL::JntArray(njO);

    KDL::Frame cartposOur;

std::cout<<"Solvinf direct task for: "<<std::endl;
    //jointpositionsO(1) = 0;
    //jointpositionsO(1) = 3.18;
    //jointpositionsO() = -3.14;
    //jointpositionsO(5) = -3.14;
#if 0
    jointpositionsO(0) = 1.57;
    jointpositionsO(1) = -1.42;
    jointpositionsO(2) = 2.29;
    jointpositionsO(3) = -1.87;
    jointpositionsO(4) = -1.18;
#endif
    jointpositionsO(0) = -5.64;
    jointpositionsO(1) = 1.02;
    jointpositionsO(2) = -2.57;
    jointpositionsO(3) = 0.09;
    jointpositionsO(4) = -.98;
    jointpositionsO(5) = -.65;

    jointpositionsO(0) = -5.644813679970512;
    jointpositionsO(1) = 1.0192200000000002;
    jointpositionsO(2) = -2.570451109167338;
    jointpositionsO(3) = 0.09110618695410988;
    jointpositionsO(4) = -0.8777609874130459;
    jointpositionsO(5) = -0.652194634885241;

    for(auto i = 0; i < njO; ++i)
    {
        std::cout<<"["<<i<<"] = "<<jointpositionsO(i)<<std::endl;
    }
    // Calculate forward position kinematics
    KDL::ChainFkSolverPos_recursive fksolverO = KDL::ChainFkSolverPos_recursive(ourChain);
    kinematics_status = fksolverO.JntToCart(jointpositionsO,cartposOur);
    if(kinematics_status>=0){
        std::cout << cartposOur <<std::endl;
        double x,y,z,w;
        cartposOur.M.GetQuaternion(x,y,z,w);
        std::cout << "Rotation:"<<x<<", "<<y<<", "<<z<<", "<<w<<std::endl;
        printf("%s \n","Succes, thanks KDL!");
    }else{
        printf("%s \n","Error: could not calculate forward kinematics :(");
    }

    cartposOur.p.data[1] += 0.05;
    KDL::ChainIkSolverVel_pinv iksolver1v(ourChain);//Inverse velocity solver
    KDL::ChainIkSolverPos_NR iksolver1(ourChain,fksolverO,iksolver1v,1000,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6

    struct timeval st, en;
 
    gettimeofday(&st, NULL);
int ret = iksolver1.CartToJnt(jointpositionsO,cartposOur,jointpositionsNew);
    gettimeofday(&en, NULL);
std::cout<<"ret = "<<ret<<std::endl;
    if (! ret) {
        std::cout<<"Solution obtained is:" <<std::endl;
        for(auto i = 0; i < ourChain.getNrOfJoints(); ++i)
            std::cout<<"\t"<<jointpositionsO(i) <<" -> "<<jointpositionsNew(i)<<std::endl;
    }

    std::cout<<"Calculation took "<<(en.tv_sec - st.tv_sec)* 1000000 + (en.tv_usec - st.tv_usec)<<" usec"<<std::endl;


    return 0;
}

