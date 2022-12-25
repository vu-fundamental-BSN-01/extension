#include <gtest/gtest.h>

#include "libbsn/goalmodel/GoalTree.hpp"
#include "libbsn/goalmodel/Goal.hpp"
#include "libbsn/goalmodel/Task.hpp"
#include "libbsn/goalmodel/LeafTask.hpp"

#include <ostream>
#include <algorithm> // std::find
#include <memory>

using namespace bsn::goalmodel;

class GoalTreeTest : public testing::Test { 
    protected:
        GoalTreeTest() {}

        virtual void SetUp() {
        }
};

TEST_F(GoalTreeTest, SimpleConstruct) {
    std::string actor = "Body Sensor Network";

    GoalTree goaltree(actor);

    ASSERT_EQ(goaltree.getActor(), actor);
}

TEST_F(GoalTreeTest, AddRootGoal) {
    std::string actor = "Body Sensor Network";
    GoalTree goaltree(actor);
    Goal rgoal("G1", "Emergency is detected");

    goaltree.addRootGoal(std::make_shared<Goal>(rgoal));

    ASSERT_TRUE(*(goaltree.getNode("G1"))==rgoal);
    ASSERT_EQ(goaltree.getSize(), 1);
}

TEST_F(GoalTreeTest, DoNotAllowMoreThanOneRootGoal) {
    std::string actor = "Body Sensor Network";
    GoalTree goaltree(actor);
    Goal rgoal1("G1", "Emergency is detected");
    Goal rgoal2("G2", "Patient is healthy");

    goaltree.addRootGoal(std::make_shared<Goal>(rgoal1));

    try {
        goaltree.addRootGoal(std::make_shared<Goal>(rgoal2));
        FAIL() << "Expected not to allow more than one root goal";
    }
    catch(std::invalid_argument const & err) {
        EXPECT_EQ(err.what(),std::string("No more than 1 root goals allowed"));
    }
}

TEST_F(GoalTreeTest, GetSpecificNode) {
    std::string actor = "Body Sensor Network";
    GoalTree goaltree(actor);
    Goal rgoal("G1", "Emergency is detected");
    goaltree.addRootGoal(std::make_shared<Goal>(rgoal));

    std::shared_ptr<Node> new_goal = goaltree.getNode("G1");

    ASSERT_EQ(Node(rgoal), *new_goal);
}

TEST_F(GoalTreeTest, GetTreeSize) {
    std::string actor = "Body Sensor Network";
    GoalTree goaltree(actor);
    Goal rgoal("G1", "Emergency is detected");
    goaltree.addRootGoal(std::make_shared<Goal>(rgoal));

    int size = goaltree.getSize();

    ASSERT_TRUE(size == 1);
}

TEST_F(GoalTreeTest, AddGoalWOneChild) {
    std::string actor = "Body Sensor Network";
    GoalTree goaltree(actor);
    Goal goal1("G1", "Emergency is detected");
    Goal goal2("G2", "Patient status is monitored");

    goal1.addChild(std::make_shared<Goal>(goal2));
    goaltree.addRootGoal(std::make_shared<Goal>(goal1));

    ASSERT_EQ(*(goaltree.getNode("G1")), goal1);
    ASSERT_EQ(*(goaltree.getNode("G2")), goal2);
    ASSERT_EQ(*(goaltree.getNode("G1")->getChild("G2")), goal2);
}

TEST_F(GoalTreeTest, AddGoalWChildren) {
    std::string actor = "Body Sensor Network";
    GoalTree goaltree(actor);
    Goal goal1("G1", "Emergency is detected");
    Goal goal2("G2", "Patient status is monitored");
    Goal goal3("G3", "Vital signs are monitored");
    Goal goal4("G4", "Vital signs are analyzed");
    Goal goal5("G5", "Weather is monitored");
    Goal goal6("G6", "Weather is analyzed");

    goal2.addChild(std::make_shared<Goal>(goal3));
    goal2.addChild(std::make_shared<Goal>(goal4));
    goal2.addChild(std::make_shared<Goal>(goal5));
    goal2.addChild(std::make_shared<Goal>(goal6));
    goal1.addChild(std::make_shared<Goal>(goal2));
    goaltree.addRootGoal(std::make_shared<Goal>(goal1));

    ASSERT_EQ(goaltree.getSize(),4);
    ASSERT_TRUE(*(goaltree.getNode("G2"))==goal2);
    ASSERT_EQ(goaltree.getNode("G2")->getChildren().size(),2);
    ASSERT_TRUE(*(goaltree.getNode("G2")->getChild("G3"))==goal3);
    ASSERT_TRUE(*(goaltree.getNode("G2")->getChild("G4"))==goal4);
    ASSERT_TRUE(*(goaltree.getNode("G2")->getChild("G5"))==goal5);
    ASSERT_TRUE(*(goaltree.getNode("G2")->getChild("G6"))==goal6);
}

TEST_F(GoalTreeTest, AddGoalWMoreChildren) {
    std::string actor = "Body Sensor Network";
    GoalTree goaltree(actor);
    Goal goal1("G1", "Emergency is detected");
    Goal goal2("G2", "Patient status is monitored");
    Goal goal3("G3", "Vital signs are monitored");
    Goal goal4("G4", "Vital signs are analyzed");
    Goal goal5("G5", "Weather is monitored");
    Goal goal6("G6", "Weather is analyzed");
    Goal goal7("G7", "Just another goal");
    Goal goal8("G8", "Other another goal");


    goal5.addChild(std::make_shared<Goal>(goal8));
    goal3.addChild(std::make_shared<Goal>(goal7));
    goal2.addChild(std::make_shared<Goal>(goal6));
    goal2.addChild(std::make_shared<Goal>(goal5));
    goal2.addChild(std::make_shared<Goal>(goal4));
    goal2.addChild(std::make_shared<Goal>(goal3));
    goal1.addChild(std::make_shared<Goal>(goal2));

    goaltree.addRootGoal(std::make_shared<Goal>(goal1));

    ASSERT_EQ(goaltree.getSize(),6);
    ASSERT_TRUE(*(goaltree.getNode("G2")->getChild("G3"))==goal3);
    //ASSERT_TRUE(goaltree.getNode("G2").getChild("G4")==goal4);
    ASSERT_TRUE(*(goaltree.getNode("G3")->getChild("G7"))==goal7);
    ASSERT_TRUE(*(goaltree.getNode("G5")->getChild("G8"))==goal8);

}

TEST_F(GoalTreeTest, AddTaskWParent) {
    std::string actor = "Body Sensor Network";
    GoalTree goaltree(actor);
    Goal goal1("G1", "Emergency is detected");
    Goal goal2("G2", "Patient status is monitored");
    Goal goal3("G3", "Vital signs are monitored");
    Goal goal4("G4", "Vital signs are analyzed");
    Goal goal5("G5", "Weather is monitored");
    Goal goal6("G6", "Weather is analyzed");

    Task task1("T1", "Analyze vital signs");
    goal4.addChild(std::make_shared<Task>(task1));
    goal2.addChild(std::make_shared<Goal>(goal4));
    goal2.addChild(std::make_shared<Goal>(goal3));
    goal1.addChild(std::make_shared<Goal>(goal2));
    goaltree.addRootGoal(std::make_shared<Goal>(goal1));

    Task task2("W1", "Analyze Weather");
    goal6.addChild(std::make_shared<Task>(task2));
    goal2.addChild(std::make_shared<Goal>(goal6));
    goal2.addChild(std::make_shared<Goal>(goal5));
    goal1.addChild(std::make_shared<Goal>(goal2));
    goaltree.addRootGoal(std::make_shared<Goal>(goal1));

    ASSERT_EQ(*(goaltree.getNode("T1")), task1);
    ASSERT_EQ(*(goaltree.getNode("G1")), goal1);
    ASSERT_EQ(*(goaltree.getNode("G2")), goal2);
    ASSERT_EQ(*(goaltree.getNode("G3")), goal3);
    ASSERT_EQ(*(goaltree.getNode("G4")), goal4);
    ASSERT_EQ(*(goaltree.getNode("G5")), goal5);
    ASSERT_EQ(*(goaltree.getNode("G6")), goal6);

    ASSERT_EQ(goaltree.getNode("G4")->getChildren().size(), 1);
    ASSERT_EQ(*(goaltree.getNode("G4")->getChild("T1")), task1);
    ASSERT_EQ(goaltree.getNode("G6")->getChildren().size(), 2);
    ASSERT_EQ(*(goaltree.getNode("G6")->getChild("T1")), task2);
    ASSERT_EQ(5, goaltree.getSize());
}

TEST_F(GoalTreeTest, AddGoalAsTaskChild) {
    std::string actor = "Body Sensor Network";
    GoalTree goaltree(actor);
    Goal goal1("G1", "Emergency is detected");
    Goal goal2("G2", "Patient status is monitored");
    Goal goal3("G3", "Vital signs are monitored");
    Goal goal4("G4", "Vital signs are analyzed");
    Goal goal5("G5", "Weather is monitored");
    Goal goal6("G6", "Weather is analyzed");
    Goal goal7("G7", "The bastard");
    Goal goal8("G8", "The bastard2");
    Task task1("T1", "Analyze vital signs");
    Task task2("T2", "Analyze weather");

    try {
        task2.addChild(std::make_shared<Goal>(goal8));
        task1.addChild(std::make_shared<Goal>(goal7));
        goal4.addChild(std::make_shared<Task>(task1));
        goal2.addChild(std::make_shared<Goal>(goal6));
        goal2.addChild(std::make_shared<Goal>(goal5));
        goal2.addChild(std::make_shared<Goal>(goal4));
        goal2.addChild(std::make_shared<Goal>(goal3));
        goal1.addChild(std::make_shared<Goal>(goal2));
        goaltree.addRootGoal(std::make_shared<Goal>(goal1));
        FAIL() << "Expected not to allow an insertion of goal as a tasks child";
    }
    catch(std::invalid_argument const & err) {
        EXPECT_EQ(err.what(),std::string("Tasks cannot contain goals as children"));
    }

    ASSERT_TRUE(true);
}

TEST_F(GoalTreeTest, AddTaskWChildren) {
    std::string actor = "Body Sensor Network";
    GoalTree goaltree(actor);
    Goal goal1("G1", "Emergency is detected");
    Goal goal2("G2", "Patient status is monitored");
    Goal goal3("G3", "Vital signs are monitored");
    Goal goal4("G4", "Vital signs are analyzed");
    Goal goal5("G5", "Weather is monitored");
    Goal goal6("G6", "Weather is  analyzed");
    Task task1("T1", "Analyze vital signs");
    Task task2("T2", "Analyze weather");
    LeafTask task111("T1.11", "Fuse sensors data", Property("W_G4_T1_11",1), Property("R_G4_T1_11",1), Property("F_G4_T1_11",1));
    LeafTask task112("T1.12", "Detect patient status", Property("W_G4_T1_12",1), Property("R_G4_T1_12",1), Property("F_G4_T1_12",1));
    LeafTask task113("T1.13", "Persist patient data", Property("W_G4_T1_13",1), Property("R_G4_T1_13",1), Property("F_G4_T1_13",1));
    LeafTask task211("T2.11", "Fuse env sensors data", Property("W_G4_T2_11",1), Property("R_G4_T2_11",1), Property("F_G4_T2_11",1));
    LeafTask task212("T2.12", "Detect weather status", Property("W_G4_T2_12",1), Property("R_G4_T2_12",1), Property("F_G4_T2_12",1));
    LeafTask task213("T2.13", "Persist weather data", Property("W_G4_T2_13",1), Property("R_G4_T2_13",1), Property("F_G4_T2_13",1));

    task2.addChild(std::make_shared<Task>(task211));
    task2.addChild(std::make_shared<Task>(task212));
    task2.addChild(std::make_shared<Task>(task213));
    task1.addChild(std::make_shared<Task>(task111));
    task1.addChild(std::make_shared<Task>(task112));
    task1.addChild(std::make_shared<Task>(task113));
    goal6.addChild(std::make_shared<Task>(task2));
    goal4.addChild(std::make_shared<Task>(task1));
    goal2.addChild(std::make_shared<Goal>(goal6));
    goal2.addChild(std::make_shared<Goal>(goal5));
    goal2.addChild(std::make_shared<Goal>(goal4));
    goal2.addChild(std::make_shared<Goal>(goal3));
    goal1.addChild(std::make_shared<Goal>(goal2));
    goaltree.addRootGoal(std::make_shared<Goal>(goal1));

    ASSERT_EQ(*(goaltree.getNode("T2.11")), task211);
    ASSERT_EQ(*(goaltree.getNode("T2.12")), task212);
    ASSERT_EQ(*(goaltree.getNode("T2.13")), task213);
    ASSERT_EQ(*(goaltree.getNode("T1.11")), task111);
    ASSERT_EQ(*(goaltree.getNode("T1.12")), task112);
    ASSERT_EQ(*(goaltree.getNode("T1.13")), task113);
    ASSERT_EQ(*(goaltree.getNode("T2")), task2);
    ASSERT_EQ(*(goaltree.getNode("T1")), task1);
    ASSERT_EQ(*(goaltree.getNode("G1")), goal1);
    ASSERT_EQ(*(goaltree.getNode("G2")), goal2);
    ASSERT_EQ(*(goaltree.getNode("G3")), goal3);
    ASSERT_EQ(*(goaltree.getNode("G4")), goal4);
    ASSERT_EQ(*(goaltree.getNode("G5")), goal5);
    ASSERT_EQ(*(goaltree.getNode("G6")), goal6);

    ASSERT_EQ(8, goaltree.getSize());
}

TEST_F(GoalTreeTest, GetLeafTasks) {
    /*Arrange*/
        std::string actor = "Body Sensor Network";
        GoalTree goaltree(actor);
        Goal goal1("G1", "Emergency is detected");
        Goal goal2("G2", "Patient status is monitored");
        Goal goal4("G4", "Vital signs are analyzed");
        Goal goal6("G6", "Weather is analyzed");
        Task task1("T1", "Analyze vital signs");
        Task task2("T2", "Analyze weather");
        LeafTask task111("T1.11", "Fuse sensors data", Property("W_G4_T1_11",1), Property("R_G4_T1_11",1), Property("F_G4_T1_11",1));
        LeafTask task112("T1.12", "Detect patient status", Property("W_G4_T1_12",1), Property("R_G4_T1_12",1), Property("F_G4_T1_12",1));
        LeafTask task113("T1.13", "Persist patient data", Property("W_G4_T1_13",1), Property("R_G4_T1_13",1), Property("F_G4_T1_13",1));
        LeafTask task114("T1.14", "Persist patient data", Property("W_G4_T1_14",1), Property("R_G4_T1_14",1), Property("F_G4_T1_14",1));     
        LeafTask task211("T2.11", "Fuse env sensors data", Property("W_G4_T2_11",1), Property("R_G4_T2_11",1), Property("F_G4_T2_11",1));
        LeafTask task212("T2.12", "Detect weather status", Property("W_G4_T2_12",1), Property("R_G4_T2_12",1), Property("F_G4_T2_12",1));
        LeafTask task213("T2.13", "Persist weather data", Property("W_G4_T2_13",1), Property("R_G4_T2_13",1), Property("F_G4_T2_13",1));
        LeafTask task214("T2.14", "Persist weather data", Property("W_G4_T2_14",1), Property("R_G4_T2_14",1), Property("F_G4_T2_14",1));     


        std::shared_ptr<LeafTask> pLeafTask111 = std::make_shared<LeafTask>(task111);
        std::shared_ptr<LeafTask> pLeafTask112 = std::make_shared<LeafTask>(task112);
        std::shared_ptr<LeafTask> pLeafTask113 = std::make_shared<LeafTask>(task113);
        std::shared_ptr<LeafTask> pLeafTask114 = std::make_shared<LeafTask>(task114);
        std::shared_ptr<LeafTask> pLeafTask211 = std::make_shared<LeafTask>(task211);
        std::shared_ptr<LeafTask> pLeafTask212 = std::make_shared<LeafTask>(task212);
        std::shared_ptr<LeafTask> pLeafTask213 = std::make_shared<LeafTask>(task213);
        std::shared_ptr<LeafTask> pLeafTask214 = std::make_shared<LeafTask>(task214);

        task1.addChild(pLeafTask111);
        task1.addChild(pLeafTask112);
        task1.addChild(pLeafTask113);
        task1.addChild(pLeafTask114);
        task2.addChild(pLeafTask211);
        task2.addChild(pLeafTask212);
        task2.addChild(pLeafTask213);
        task2.addChild(pLeafTask214);
        goal6.addChild(std::make_shared<Task>(task2));
        goal4.addChild(std::make_shared<Task>(task1));
        goal2.addChild(std::make_shared<Goal>(goal6));
        goal2.addChild(std::make_shared<Goal>(goal4));
        goal1.addChild(std::make_shared<Goal>(goal2));
        goaltree.addRootGoal(std::make_shared<Goal>(goal1));
    
    /*Act*/
        std::vector<std::shared_ptr<LeafTask>> LTvec = goaltree.getLeafTasks();

    /*Assert*/
        ASSERT_EQ(4 ,LTvec.size());

        ASSERT_EQ(*(std::find(LTvec.begin(), LTvec.end(), pLeafTask111)), pLeafTask111);
        ASSERT_EQ(*(std::find(LTvec.begin(), LTvec.end(), pLeafTask112)), pLeafTask112);
        ASSERT_EQ(*(std::find(LTvec.begin(), LTvec.end(), pLeafTask113)), pLeafTask113);
        ASSERT_EQ(*(std::find(LTvec.begin(), LTvec.end(), pLeafTask114)), pLeafTask114);
        ASSERT_EQ(*(std::find(LTvec.begin(), LTvec.end(), pLeafTask211)), pLeafTask211);
        ASSERT_EQ(*(std::find(LTvec.begin(), LTvec.end(), pLeafTask212)), pLeafTask212);
        ASSERT_EQ(*(std::find(LTvec.begin(), LTvec.end(), pLeafTask213)), pLeafTask213);
        ASSERT_EQ(*(std::find(LTvec.begin(), LTvec.end(), pLeafTask214)), pLeafTask214);
}