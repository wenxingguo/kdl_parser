#include "kdl_parser/kdl_parser.hpp"
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames_io.hpp>

int main(int argc, char* argv[])
{
    urdf::Model abb_model;
    abb_model.initFile("./urdf/irb6700_200_260.urdf");

    KDL::Tree abb_tree;
    kdl_parser::treeFromUrdfModel(abb_model, abb_tree);
    // 工具位姿 491.158,70.0335,463.14
    KDL::Frame grinding_head_tool_tip({ 491.158 / 1000, 70.0335 / 1000, 463.14 / 1000 });
    abb_tree.addSegment(KDL::Segment("grinding_head", KDL::Joint(), grinding_head_tool_tip), "tool0");

    KDL::Chain abb_chain;
    abb_tree.getChain("base_link", "grinding_head", abb_chain);

    std::cout << abb_chain.getNrOfJoints() << std::endl;

    KDL::JntArray q0(abb_chain.getNrOfJoints());
    KDL::JntArray q_init(abb_chain.getNrOfJoints());
    q0(0) = 100 * KDL::deg2rad;
    q0(1) = 50 * KDL::deg2rad;
    q0(2) = 20 * KDL::deg2rad;
    q0(3) = 0;
    q0(4) = 20 * KDL::deg2rad;
    q0(5) = 0;

    q_init.data << 0.1, 0.1, 0, 0, 0, 0;

    // 正运动学解
    std::shared_ptr<KDL::ChainFkSolverPos> fk_solver(new KDL::ChainFkSolverPos_recursive(abb_chain));
    KDL::Frame T;
    fk_solver->JntToCart(q0, T);
    std::cout << T << std::endl;

    // 逆运动学解
    KDL::JntArray q0_inv(abb_chain.getNrOfJoints());
    std::shared_ptr<KDL::ChainIkSolverPos> ik_solver(new KDL::ChainIkSolverPos_LMA(abb_chain));
    ik_solver->CartToJnt(q_init, T, q0_inv);
    std::cout << q0.data.transpose() << std::endl;
    std::cout << q0_inv.data.transpose() << std::endl;

    // 雅可比举证
    KDL::Jacobian J(6);
    std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver(new KDL::ChainJntToJacSolver(abb_chain));
    jac_solver->JntToJac(q0, J);
    std::cout << J.data << std::endl;

    return 0;
}
