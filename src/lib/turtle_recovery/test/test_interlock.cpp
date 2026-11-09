#include "../ArmingInterlock.hpp"
#include <cassert>
#include <atomic>
#include <thread>
#include <iostream>
using namespace turtle;
Interlock *other_translation_unit();
int main(){
 assert(other_translation_unit()==&interlock());
 uint8_t state=1;
 {ArmGuard a(state,2,true);assert(!a.allowed());}
 interlock().set_ready(true);
 {ArmGuard a(state,2,true);assert(a.allowed());assert(!interlock().acquire_recovery());} // failed arm frees reservation
 assert(interlock().owner()==Owner::NONE);
 {ArmGuard a(state,2,true);assert(a.allowed());state=2;}
 assert(interlock().owner()==Owner::FLIGHT);
 {DisarmGuard a(state,2);state=1;}
 assert(interlock().owner()==Owner::NONE);
 assert(interlock().acquire_recovery());{ArmGuard a(state,2,false);assert(!a.allowed());}
 interlock().release_recovery();
 Interlock lock;lock.set_ready(true);std::atomic<int> owner_count{0};
 auto flight=[&](){for(int i=0;i<100000;++i)if(lock.acquire_flight(true)){assert(owner_count.fetch_add(1)==0);assert(owner_count.fetch_sub(1)==1);lock.release_flight();}};
 auto recovery=[&](){for(int i=0;i<100000;++i)if(lock.acquire_recovery()){assert(owner_count.fetch_add(1)==0);assert(owner_count.fetch_sub(1)==1);lock.release_recovery();}};
 std::thread a(flight),b(recovery);a.join();b.join();
 std::cout<<"PASS: Commander guard lifetimes, cross-translation-unit identity, 200000 concurrent reservation attempts\n";
}
