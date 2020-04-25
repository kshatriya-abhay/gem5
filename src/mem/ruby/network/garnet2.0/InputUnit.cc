/*
 * Copyright (c) 2008 Princeton University
 * Copyright (c) 2016 Georgia Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Niket Agarwal
 *          Tushar Krishna
 */


#include "mem/ruby/network/garnet2.0/InputUnit.hh"

#include "base/stl_helpers.hh"
#include "debug/RubyNetwork.hh"
#include "mem/ruby/network/garnet2.0/Credit.hh"
#include "mem/ruby/network/garnet2.0/Router.hh"

using namespace std;
using m5::stl_helpers::deletePointers;

InputUnit::InputUnit(int id, PortDirection direction, Router *router)
            : Consumer(router)
{
    m_id = id;
    m_direction = direction;
    m_router = router;
    m_num_vcs = m_router->get_num_vcs();
    m_vc_per_vnet = m_router->get_vc_per_vnet();

    m_num_buffer_reads.resize(m_num_vcs/m_vc_per_vnet);
    m_num_buffer_writes.resize(m_num_vcs/m_vc_per_vnet);
    for (int i = 0; i < m_num_buffer_reads.size(); i++) {
        m_num_buffer_reads[i] = 0;
        m_num_buffer_writes[i] = 0;
    }

    creditQueue = new flitBuffer();
    // Instantiating the virtual channels
    m_vcs.resize(m_num_vcs);
    for (int i=0; i < m_num_vcs; i++) {
        m_vcs[i] = new VirtualChannel(i);
    }
}

InputUnit::~InputUnit()
{
    delete creditQueue;
    deletePointers(m_vcs);
}

/*
 * The InputUnit wakeup function reads the input flit from its input link.
 * Each flit arrives with an input VC.
 * For HEAD/HEAD_TAIL flits, performs route computation,
 * and updates route in the input VC.
 * The flit is buffered for (m_latency - 1) cycles in the input VC
 * and marked as valid for SwitchAllocation starting that cycle.
 *
 */

void
InputUnit::wakeup()
{
    flit *t_flit;
    int outport = -1;
    int M5_VAR_USED num_rows = m_router->get_net_ptr()->getNumRows();
    int num_cols = m_router->get_net_ptr()->getNumCols();
    
    if (m_in_link->isReady(m_router->curCycle())) {

        t_flit = m_in_link->consumeLink();
        int vc = t_flit->get_vc();
        int my_id = m_router->get_id();
        t_flit->increment_hops(); // for stats
        if (t_flit->get_type() == DEBUG_) {
            int h = t_flit->get_route().hops_traversed;
            printf("  DEBUG FLIT. Hops = %d\n", h);
            assert(m_vcs[vc]->get_state() == IDLE_);
            set_vc_active(vc, m_router->curCycle());
            if(h > 1){
                printf("This is a router neighbouring the faulty router.\n");
            }
            else if(h == 1){
                if(t_flit->get_route().dest_router == my_id){
                    //if we have already sent a debug pulse, we stop here.
                    delete t_flit;
                    //its job is done
                    if(m_router->m_debug_pulses == 1){
                        m_router->schedule_wakeup(Cycles( m_router->curCycle() + Cycles(1) )); //wakeup the router in the next cycle
                    }
                    else{
                        //send flits and mark that we have sent flits once.
                        //generate flits
                        if(my_id != (num_cols*num_rows)-1){
                            //schedule a flit to send right
                            RouteInfo rt_right;
                            rt_right.vnet = vc/m_vc_per_vnet;
                            // rt_right.net_dest = new_net_msg_ptr->getDestination();
                            // rt_right.src_ni = m_id;
                            rt_right.src_router = my_id;
                            // rt_right.dest_ni = destID;
                            rt_right.dest_router = my_id + 1;

                            int fid = my_id*(-100) + (-1);  //will help to identify the flit
                            flit *fl_right = new flit(fid, -1, vc, vc/m_vc_per_vnet, rt_right, 1, NULL,
                            m_router->curCycle());
                            m_in_link->flit_push_back_debug(fl_right);
                        }
                        if(my_id != num_cols*(num_rows-1)){
                            //schedule a flit to send left
                            RouteInfo rt_left;
                            rt_left.vnet = vc/m_vc_per_vnet;
                            // rt_left.net_dest = new_net_msg_ptr->getDestination();
                            // rt_left.src_ni = m_id;
                            rt_left.src_router = my_id;
                            // rt_left.dest_ni = destID;
                            rt_left.dest_router = my_id - 1;

                            int fid = my_id*(-100) + (-2);  //will help to identify the flit
                            flit *fl_left = new flit(fid, -2, vc, vc/m_vc_per_vnet, rt_left, 1, NULL,
                            m_router->curCycle());
                            m_in_link->flit_push_back_debug(fl_left);
                        }
                        if(my_id >= num_cols){
                            //send a flit to bottom
                            RouteInfo rt_bottom;
                            rt_bottom.vnet = vc/m_vc_per_vnet;
                            // rt_bottom.net_dest = new_net_msg_ptr->getDestination();
                            // rt_bottom.src_ni = m_id;
                            rt_bottom.src_router = my_id;
                            // rt_bottom.dest_ni = destID;
                            rt_bottom.dest_router = my_id - num_cols;

                            int fid = my_id*(-100) + (-3);  //will help to identify the flit
                            flit *fl_bottom = new flit(fid, -3, vc, vc/m_vc_per_vnet, rt_bottom, 1, NULL,
                            m_router->curCycle());
                            outport = m_router->route_compute(fl_bottom, m_id, m_direction);
                            grant_outport(vc, outport);
                            m_vcs[vc]->insertFlit(t_flit);


                            int vnet = vc/m_vc_per_vnet;
                            // number of writes same as reads
                            // any flit that is written will be read only once
                            m_num_buffer_writes[vnet]++;
                            m_num_buffer_reads[vnet]++;

                            Cycles pipe_stages = m_router->get_pipe_stages();
                            if (pipe_stages == 1) {
                                // 1-cycle router
                                // Flit goes for SA directly
                                t_flit->advance_stage(SA_, m_router->curCycle());
                            } else {
                                assert(pipe_stages > 1);
                                // Router delay is modeled by making flit wait in buffer for
                                // (pipe_stages cycles - 1) cycles before going for SA

                                Cycles wait_time = pipe_stages - Cycles(1);
                                fl_bottom->advance_stage(SA_, m_router->curCycle() + wait_time);

                                // Wakeup the router in that cycle to perform SA
                                m_router->schedule_wakeup(Cycles(wait_time));
                            }
                        }
                        m_router->m_debug_pulses = 1;
                        return;
                    }
                }
                else{
                    // route_compute and send it
                    outport = m_router->route_compute(t_flit,
                        m_id, m_direction);
                    grant_outport(vc, outport);
                    m_vcs[vc]->insertFlit(t_flit);
                    int vnet = vc/m_vc_per_vnet;
                    // number of writes same as reads
                    // any flit that is written will be read only once
                    m_num_buffer_writes[vnet]++;
                    m_num_buffer_reads[vnet]++;

                    Cycles pipe_stages = m_router->get_pipe_stages();
                    if (pipe_stages == 1) {
                        // 1-cycle router
                        // Flit goes for SA directly
                        t_flit->advance_stage(SA_, m_router->curCycle());
                    } else {
                        assert(pipe_stages > 1);
                        // Router delay is modeled by making flit wait in buffer for
                        // (pipe_stages cycles - 1) cycles before going for SA

                        Cycles wait_time = pipe_stages - Cycles(1);
                        t_flit->advance_stage(SA_, m_router->curCycle() + wait_time);

                        // Wakeup the router in that cycle to perform SA
                        m_router->schedule_wakeup(Cycles(wait_time));
                    }
                    return;
                }
            }

        }
        if ((t_flit->get_type() == HEAD_) ||
            (t_flit->get_type() == HEAD_TAIL_)) {

            assert(m_vcs[vc]->get_state() == IDLE_);
            set_vc_active(vc, m_router->curCycle());

            // Route computation for this vc
            // printf("gid %d\t",t_flit->get_gid());

            RouteInfo route = t_flit->get_route();
            
            int fault_router_id = m_router->get_net_ptr()->getfaultrouter();
            if(my_id == fault_router_id){
                printf("infecting at fault router: %d\n",fault_router_id);
                route.dest_router = (num_rows * num_cols) + (route.dest_router % num_cols);
                t_flit->set_route(route);
            }

            outport = m_router->route_compute(t_flit,
                m_id, m_direction);
            if(outport == -1){  /* Instead of re-insertion,
                                   we can just accept the packet at this router, drop it,
                                   and then send detection signals*/

                // route.dest_router = my_id;
                // t_flit->set_route(route);
                // printf("VC Number: %d\n", vc);
                // t_flit->set_vc(vc);
                printf("Accepting a bad flit at the edge router %d\n", my_id);
                update_blocked_vcs(vc); // for displaying
                // outport = m_router->route_compute(t_flit, m_id, m_direction);
                
                //generate flits
                if(my_id != (num_cols*num_rows)-1){
                    //schedule a flit to send right
                    RouteInfo rt_right;
                    rt_right.vnet = vc/m_vc_per_vnet;
                    // rt_right.net_dest = new_net_msg_ptr->getDestination();
                    // rt_right.src_ni = m_id;
                    rt_right.src_router = my_id;
                    // rt_right.dest_ni = destID;
                    rt_right.dest_router = my_id + 1;

                    int fid = my_id*(-100) + (-1);  //will help to identify the flit
                    flit *fl_right = new flit(fid, -1, vc, vc/m_vc_per_vnet, rt_right, 1, NULL,
                    m_router->curCycle());
                    m_in_link->flit_push_back_debug(fl_right);
                }
                if(my_id != num_cols*(num_rows-1)){
                    //schedule a flit to send left
                    RouteInfo rt_left;
                    rt_left.vnet = vc/m_vc_per_vnet;
                    // rt_left.net_dest = new_net_msg_ptr->getDestination();
                    // rt_left.src_ni = m_id;
                    rt_left.src_router = my_id;
                    // rt_left.dest_ni = destID;
                    rt_left.dest_router = my_id - 1;

                    int fid = my_id*(-100) + (-2);  //will help to identify the flit
                    flit *fl_left = new flit(fid, -2, vc, vc/m_vc_per_vnet, rt_left, 1, NULL,
                    m_router->curCycle());
                    m_in_link->flit_push_back_debug(fl_left);
                }
                //send a flit to bottom
                RouteInfo rt_bottom;
                rt_bottom.vnet = vc/m_vc_per_vnet;
                // rt_bottom.net_dest = new_net_msg_ptr->getDestination();
                // rt_bottom.src_ni = m_id;
                rt_bottom.src_router = my_id;
                // rt_bottom.dest_ni = destID;
                rt_bottom.dest_router = my_id - num_cols;

                int fid = my_id*(-100) + (-3);  //will help to identify the flit
                flit *fl_bottom = new flit(fid, -3, vc, vc/m_vc_per_vnet, rt_bottom, 1, NULL,
                m_router->curCycle());
                outport = m_router->route_compute(fl_bottom, m_id, m_direction);
                grant_outport(vc, outport);
                m_vcs[vc]->insertFlit(t_flit);

                m_router->m_debug_pulses = 1;

                int vnet = vc/m_vc_per_vnet;
                // number of writes same as reads
                // any flit that is written will be read only once
                m_num_buffer_writes[vnet]++;
                m_num_buffer_reads[vnet]++;

                Cycles pipe_stages = m_router->get_pipe_stages();
                if (pipe_stages == 1) {
                    // 1-cycle router
                    // Flit goes for SA directly
                    t_flit->advance_stage(SA_, m_router->curCycle());
                } else {
                    assert(pipe_stages > 1);
                    // Router delay is modeled by making flit wait in buffer for
                    // (pipe_stages cycles - 1) cycles before going for SA

                    Cycles wait_time = pipe_stages - Cycles(1);
                    fl_bottom->advance_stage(SA_, m_router->curCycle() + wait_time);

                    // Wakeup the router in that cycle to perform SA
                    m_router->schedule_wakeup(Cycles(wait_time));
                }
                return;
            }
            // Update output port in VC
            // All flits in this packet will use this output port
            // The output port field in the flit is updated after it wins SA
            
            grant_outport(vc, outport);
            
        } else {
            assert(m_vcs[vc]->get_state() == ACTIVE_);
            // if(outport == -1){
            //     outport = 2;
            // }
        }

            // Buffer the flit
            m_vcs[vc]->insertFlit(t_flit);
            // printf("Inserted the flit in VC %d\n",vc);
            int vnet = vc/m_vc_per_vnet;
            // number of writes same as reads
            // any flit that is written will be read only once
            m_num_buffer_writes[vnet]++;
            m_num_buffer_reads[vnet]++;

        if(outport != -1){
            Cycles pipe_stages = m_router->get_pipe_stages();
            if (pipe_stages == 1) {
                // 1-cycle router
                // Flit goes for SA directly
                t_flit->advance_stage(SA_, m_router->curCycle());
            } else {
                assert(pipe_stages > 1);
                // Router delay is modeled by making flit wait in buffer for
                // (pipe_stages cycles - 1) cycles before going for SA

                Cycles wait_time = pipe_stages - Cycles(1);
                t_flit->advance_stage(SA_, m_router->curCycle() + wait_time);

                // Wakeup the router in that cycle to perform SA
                m_router->schedule_wakeup(Cycles(wait_time));
            }
        }
    }
    // show_blocked_vcs();
}

void
InputUnit::update_blocked_vcs(int vc){
    int found = 0;
    for(int i=0; i<m_blocked_vcs.size(); i++){
        if(m_blocked_vcs[i]->get_id() == vc) {found = 1; break;}
    }
    if(!found){
        m_blocked_vcs.push_back(m_vcs[vc]);
    }
}

void
InputUnit::show_blocked_vcs()
{
    // std::fstream f;
    int M5_VAR_USED num_rows = m_router->get_net_ptr()->getNumRows();
    int num_cols = m_router->get_net_ptr()->getNumCols();
            
    if(m_router->get_id() < num_cols*(num_rows - 1)) return;
    if(m_blocked_vcs.size() > 0)
        {
            printf("  InputUnit: %d\n", m_id);
            for(int v=0; v < m_blocked_vcs.size(); v++){
                if(!m_blocked_vcs[v]->isEmpty()){
                    flit* t_flit = m_blocked_vcs[v]->peekTopFlit();    
                    if(t_flit->get_route().dest_router >= num_rows*num_cols){
                    printf("    VC %d, ", v);
                    printf("    Head flit: %d, dest: %d\n",
                        t_flit->get_gid(), 
                        t_flit->get_route().dest_router);
                    }
                }
            }
            printf("  VCs still available: %d\n", (int)(m_vcs.size() - m_blocked_vcs.size()));
        }
}

// Send a credit back to upstream router for this VC.
// Called by SwitchAllocator when the flit in this VC wins the Switch.
void
InputUnit::increment_credit(int in_vc, bool free_signal, Cycles curTime)
{
    Credit *t_credit = new Credit(in_vc, free_signal, curTime);
    creditQueue->insert(t_credit);
    m_credit_link->scheduleEventAbsolute(m_router->clockEdge(Cycles(1)));
}


uint32_t
InputUnit::functionalWrite(Packet *pkt)
{
    uint32_t num_functional_writes = 0;
    for (int i=0; i < m_num_vcs; i++) {
        num_functional_writes += m_vcs[i]->functionalWrite(pkt);
    }

    return num_functional_writes;
}

void
InputUnit::resetStats()
{
    for (int j = 0; j < m_num_buffer_reads.size(); j++) {
        m_num_buffer_reads[j] = 0;
        m_num_buffer_writes[j] = 0;
    }
}
