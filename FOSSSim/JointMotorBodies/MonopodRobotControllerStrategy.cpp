#include "MonopodRobotControllerStrategy.h"
#include "JointMotorBodySimulation.h"
#include <queue>
#include <ctime>
#include <sstream>

MonopodRobotControllerStrategy::Action MonopodRobotControllerStrategy::generateAction(JointMotorBodyScene * scene,  MonopodRobot * robot)
{
    scalar dt = scene->dt();
    static scalar T = 0;
    T += dt;
    static scalar theta = 0;
    static int qwe = 0;
            
    srand((unsigned)time(0));
    static int prev_y = 0;
    
    for(int i=0;i<5 && qwe == 0; i++){        
        int x = rand()%13-6;
        int y = (rand()%3) +2;

        prev_y += y;
        
        Vector2s new_v(0,0);
        VectorXs new_vertices(8);
        new_vertices(0) = x-5;
        new_vertices(1) = prev_y+0.2;
        new_vertices(2) = x+5;
        new_vertices(3) = prev_y+0.2;
        new_vertices(4) = x+5;
        new_vertices(5) = prev_y-0.2;
        new_vertices(6) = x-5;
        new_vertices(7) = prev_y-0.2;
        
        VectorXs new_masses(4);
        new_masses(0) = 1.0;
        new_masses(1) = 1.0;
        new_masses(2) = 1.0;
        new_masses(3) = 1.0;
        {
            RigidBody::RigidBody new_rb(new_v,0,new_vertices,new_masses,0.2,1);
            new_rb.set_collision_enable(false);
            new_rb.set_dir(rand()%3-1);
            new_rb.set_max_x(x+3);
            new_rb.set_min_x(x-3);
            new_rb.is_fragile = rand()%3-1;
            new_rb.has_touched = false;
            jumping_detectors[0]->set_ymin(prev_y+0.5);
            scene->addRigidBody(new_rb);  
        }      
        
        if(x-5-3-(-14) > 8){
            int tmp = (x-5-3-(-14))/2;
            x = (x-5-3-(-14))/2 - 14;
            int tmpran = rand()%2+1;
            new_vertices(0) = x-tmp+tmpran;
            new_vertices(2) = x+tmp-tmpran;
            new_vertices(4) = x+tmp-tmpran;
            new_vertices(6) = x-tmp+tmpran;
            {
                RigidBody::RigidBody new_rb(new_v,0,new_vertices,new_masses,0.2,1);
                new_rb.set_collision_enable(false);
                new_rb.set_dir(0);
                new_rb.set_max_x(x+3);
                new_rb.set_min_x(x-3);
                new_rb.is_fragile = rand()%3-1;
                new_rb.has_touched = false;
                scene->addRigidBody(new_rb); 
                
            }
        }else if(14-(x+5+3)>8){
            int tmp = (14-x-5-3)/2;
            x = 14-(14-x-5-3)/2;
            
            int tmpran = rand()%2+1;
            new_vertices(0) = x-tmp+tmpran;
            new_vertices(2) = x+tmp-tmpran;
            new_vertices(4) = x+tmp-tmpran;
            new_vertices(6) = x-tmp+tmpran;
            
            {
                RigidBody::RigidBody new_rb(new_v,0,new_vertices,new_masses,0.2,1);
                new_rb.set_collision_enable(false);
                new_rb.set_dir(0);
                new_rb.set_max_x(x+3);
                new_rb.set_min_x(x-3);
                new_rb.is_fragile = rand()%3-1;
                new_rb.has_touched = false;
                scene->addRigidBody(new_rb); 
            }
        }
    }
    
    qwe++;
    

            
    // output of the strategy
    Action action;
    
    // head and foot rigid bodies, their positions and velocities
    RigidBody * head = robot->rigidBody(robot->head());
    RigidBody * foot = robot->rigidBody(robot->foot());
    
    
    
    Vector2s head_pos = head->getX();
    Vector2s foot_pos = foot->getX();
    Vector2s head_vel = head->getV();
    Vector2s foot_vel = foot->getV();
    Vector2s gravity = scene->gravity();
    scalar g = -1.0*gravity.y();
    scalar r_foot = foot->getRadius();
    
    //std::cout << "foot_pos: " << foot_pos.y() << std::endl;
    
    int cur_percent = foot_pos.y()*100.0/(prev_y+0.5);
    if(cur_percent > 100){
        cur_percent = 100;
    }
    
    std::stringstream ss;
    ss << cur_percent;
    std::string cur_p = ss.str();
    
    int time_rem = 30.0-T;
    std::stringstream gg;
    gg << time_rem;
    std::string cur_tm = gg.str();
    
    if(time_rem < 0){
        exit(1);
        time_rem = 0;
    }
            
    
    jumping_detectors[0]->set_button(cur_p+"%  "+cur_tm+"s");
    
    
    //std::cout << scene->getRigidBodies().at(2).get_collision() << std::endl;
    
    for(int i=4;i<scene->getRigidBodies().size();i++){
        
        
        RigidBody &rb = scene->getRigidBodies().at(i);
        if(rb.getX().x() > rb.get_max_x()){
            rb.set_dir(-1);
        }
        if(rb.getX().x() < rb.get_min_x()){
            rb.set_dir(1);
        }
        //std::cout << "123123: "  << i << std::endl;
        
        rb.move_one_step();
        
        //std::cout << "123123: "  << i << std::endl;
        
        if(rb.is_fragile == 1 && rb.has_touched){
            //std::cout << rb.timer << std::endl;
            if(rb.timer <= 0.0){
                scene->getRigidBodies().at(i).set_collision_enable(false);
            }else{
                rb.timer -= 0.03;
            }
        }
        else if(foot_pos.y()-r_foot > rb.getX().y()+0.2+0.1+0.2){
            scene->getRigidBodies().at(i).set_collision_enable(true);
        }
        else if(foot_pos.y()-r_foot < rb.getX().y()-0.2){
            scene->getRigidBodies().at(i).set_collision_enable(false);
        }
    }
    //std::vector<RigidBody>& getRigidBodies();
    
    //std::cout << "wtf"<< std::endl;
    

    

    
    // leg spring rest length limits
    scalar ls_k = robot->legSpringForce()->k();
    scalar lc = robot->legSpringLength();
    scalar l0 = robot->legSpringRestLength();
    scalar l0min = robot->legSpringRestLengthMin();
    scalar l0max = robot->legSpringRestLengthMax();
    scalar ld = (l0min + l0max) / 2;
    
    ld = 2;
    
    // write to these two variables as output
    action.legSpringRestLength = l0;
    action.legTorque = 0;
    
    
    char cur_key = scene->get_key();
    int buf = scene->get_buf();
    bool has_key;
    if(buf > 0 && (cur_key=='a' ||cur_key=='s' || cur_key == 'd' ||cur_key=='w')){
        has_key = true;
    }else{
        has_key = false;
    }
    if(buf>0){
        scene->set_buf(buf-1);
    }
    
    if(has_key && cur_key == 'a'){
        action.legTorque -= 200;
    }
    
    if(has_key && cur_key == 'd'){
        action.legTorque += 200;
    }
    
    
    if(!foot->get_collision()){
            action.legSpringRestLength += 0.5*(ld-l0);
    }else{
        if(has_key && cur_key == 'w'){
                action.legSpringRestLength += 2.0;
        }
        if(has_key && cur_key == 's'){
            action.legSpringRestLength -= 2.0;
        }
    }
    
    
    
    Vector2s rela_vel;
    Vector2s rela_pos;
    rela_vel = head_vel- foot_vel;
    rela_pos = head_pos - foot_pos;

    Vector2s dir_90;
    dir_90.x() = rela_pos.y();
    dir_90.y() = -1.0*rela_pos.x();
    
    scalar vel_proj = dir_90.dot(rela_vel)/dir_90.norm();
    
    
    action.legTorque += 100*vel_proj;
    
    
    
    return action;
}
