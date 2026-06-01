// headless_micro.cpp — tiny controlled tests for settling / restitution / energy.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <hop/hop.h>
#include <vector>

using T = float;
using tr = hop::scalar_traits<T>;

static std::shared_ptr<hop::solid<T>> floor_solid(hop::simulator<T> & sim, float cor = 1.0f) {
	auto w = std::make_shared<hop::solid<T>>();
	w->set_infinite_mass();
	w->set_coefficient_of_gravity(0);
	w->set_coefficient_of_restitution(cor);
	w->set_coefficient_of_static_friction(0);
	w->set_coefficient_of_dynamic_friction(0);
	w->add_shape(std::make_shared<hop::shape<T>>(hop::aa_box<T>({-50,-50,-1},{50,50,0})));
	w->set_position({0,0,0});
	sim.add_solid(w);
	return w;
}

static std::shared_ptr<hop::solid<T>> ball(hop::simulator<T> & sim, hop::vec3<T> pos, float r=0.4f, float cor=0.5f) {
	auto s = std::make_shared<hop::solid<T>>();
	s->set_mass(1.0f);
	s->set_coefficient_of_restitution_override(true);
	s->set_coefficient_of_restitution(cor);
	s->set_coefficient_of_static_friction(0);
	s->set_coefficient_of_dynamic_friction(0);
	s->add_shape(std::make_shared<hop::shape<T>>(hop::sphere<T>(r)));
	s->set_position(pos);
	sim.add_solid(s);
	return s;
}

static void test_single_drop() {
	printf("\n=== single ball drop, COR=0.5, drop from z=5, r=0.4 ===\n");
	hop::simulator<T> sim;
	floor_solid(sim);
	auto b = ball(sim, {0,0,5}, 0.4f, 0.5f);
	float peak_after_bounce = 0;
	bool bounced = false;
	float prev_z = 5;
	for (int t = 1; t <= 600; ++t) {
		sim.update(1.0f/60.0f);
		auto p = b->get_position(); auto v = b->get_velocity();
		// detect first bounce (moving up after having been near floor)
		if (!bounced && v.z > 0.1f && p.z < 1.5f) bounced = true;
		if (bounced && v.z <= 0 && p.z > peak_after_bounce) {} // captured below
		if (bounced) peak_after_bounce = std::max(peak_after_bounce, p.z);
		if (t % 30 == 0)
			printf("t=%3d z=%.4f vz=%.4f active=%d\n", t, p.z, v.z, (int)b->active());
		prev_z = p.z;
	}
	// Expected: drop height above rest ~ (5-0.4)=4.6. After COR=0.5 bounce, rebound
	// height ~ 0.5^2 * 4.6 = 1.15 above rest => peak z ~ 1.55.
	printf("peak after first bounce: z=%.3f (ideal ~1.55 for COR=0.5)\n", peak_after_bounce);
}

static void test_stack(int N) {
	printf("\n=== vertical stack settle: %d balls, COR=0.5, r=0.4 ===\n", N);
	hop::simulator<T> sim;
	sim.set_solver_iterations(32);
	floor_solid(sim);
	std::vector<std::shared_ptr<hop::solid<T>>> bs;
	for (int i = 0; i < N; ++i) bs.push_back(ball(sim, {0,0,0.4f + i*0.85f + 0.5f}, 0.4f, 0.5f));
	for (int t = 1; t <= 900; ++t) {
		sim.update(1.0f/60.0f);
		if (t % 60 == 0 || t==1) {
			float ke=0; int active=0;
			for (auto&b:bs){auto v=b->get_velocity(); ke+=0.5f*(v.x*v.x+v.y*v.y+v.z*v.z); if(b->active())active++;}
			printf("t=%3d KE=%.4f active=%d  z=[", t, ke, active);
			for (auto&b:bs) printf("%.3f ", b->get_position().z);
			printf("]\n");
		}
	}
	// ideal settled centers: 0.4, 1.2, 2.0, ... (touching, diameter 0.8)
}

static void box_walls(hop::simulator<T> & sim, float half, float h) {
	auto w=[&](hop::vec3<T> mn, hop::vec3<T> mx, hop::vec3<T> p){
		auto s=std::make_shared<hop::solid<T>>(); s->set_infinite_mass(); s->set_coefficient_of_gravity(0);
		s->set_coefficient_of_restitution(1.0f); s->set_coefficient_of_static_friction(0); s->set_coefficient_of_dynamic_friction(0);
		s->add_shape(std::make_shared<hop::shape<T>>(hop::aa_box<T>(mn,mx))); s->set_position(p); sim.add_solid(s);
	};
	float o=half+1;
	w({-o,-o,-1},{o,o,0},{0,0,0});          // floor
	w({-o,-o,0},{o,o,1},{0,0,h});           // ceiling
	w({-1,-half,0},{0,half,h},{-half,0,0}); // -X
	w({0,-half,0},{1,half,h},{half,0,0});   // +X
	w({-half,-1,0},{half,0,h},{0,-half,0}); // -Y
	w({-half,0,0},{half,1,h},{0,half,0});   // +Y
}

static void test_box(int n, float cor) {
	float r=0.28f, spacing=r*2.2f; int half=3; float h=8;
	printf("\n=== dense box: %d^3 grid, COR=%.2f, r=%.2f ===\n", n, cor, r);
	hop::simulator<T> sim; sim.set_deactivate_count(32);
	box_walls(sim, half, h);
	std::vector<std::shared_ptr<hop::solid<T>>> bs;
	int i=0;
	for(int z=0;z<n;z++)for(int y=0;y<n;y++)for(int x=0;x<n;x++,i++){
		auto s=ball(sim,{0,0,0},r,cor); s->set_coefficient_of_restitution_override(true);
		float px=-(half-spacing*0.5f)+x*spacing, py=-(half-spacing*0.5f)+y*spacing, pz=spacing*0.5f+z*spacing;
		s->set_position({px,py,pz});
		float vx=((i*7+3)%21-10)*0.5f, vy=((i*13+5)%21-10)*0.5f, vz=((i*17+9)%21-10)*0.5f;
		s->set_velocity({vx,vy,vz});
		bs.push_back(s);
	}
	for(int t=1;t<=1200;t++){
		sim.update(1.0f/60.0f);
		if(t%60==0||t==1){
			float ke=0,maxz=0,pe=0; int active=0;
			for(auto&b:bs){auto v=b->get_velocity();auto p=b->get_position();ke+=0.5f*(v.x*v.x+v.y*v.y+v.z*v.z);pe+=10*p.z;maxz=std::max(maxz,p.z);if(b->active())active++;}
			printf("t=%4d KE=%9.2f PE=%9.2f E=%9.2f active=%4d maxZ=%.3f\n",t,ke,pe,ke+pe,active,maxz);
		}
	}
}

static void test_gas(int n, float cor) {
	// Elastic gas: gravity OFF, COR=cor, frictionless. Ideal physics conserves
	// kinetic energy exactly. Any drift in KE is pure numerical injection/loss.
	float r=0.28f, spacing=r*2.2f; int half=3; float h=8;
	printf("\n=== elastic gas (no gravity): %d^3, COR=%.2f ===\n", n, cor);
	hop::simulator<T> sim({0,0,0});
	box_walls(sim, half, h);
	std::vector<std::shared_ptr<hop::solid<T>>> bs;
	int i=0;
	for(int z=0;z<n;z++)for(int y=0;y<n;y++)for(int x=0;x<n;x++,i++){
		auto s=ball(sim,{0,0,0},r,cor); s->set_coefficient_of_restitution_override(true);
		float px=-(half-spacing*0.5f)+x*spacing, py=-(half-spacing*0.5f)+y*spacing, pz=spacing*0.5f+z*spacing;
		s->set_position({px,py,pz});
		float vx=((i*7+3)%21-10)*0.5f, vy=((i*13+5)%21-10)*0.5f, vz=((i*17+9)%21-10)*0.5f;
		s->set_velocity({vx,vy,vz});
		bs.push_back(s);
	}
	for(int t=1;t<=600;t++){
		sim.update(1.0f/60.0f);
		if(t%60==0||t==1){
			float ke=0,maxv=0; for(auto&b:bs){auto v=b->get_velocity();float s2=v.x*v.x+v.y*v.y+v.z*v.z;ke+=0.5f*s2;maxv=std::max(maxv,std::sqrt(s2));}
			printf("t=%4d KE=%9.2f maxV=%.3f\n",t,ke,maxv);
		}
	}
}

static void test_wedge(float cor) {
	// One dynamic ball settling into a frictionless V between two fixed balls.
	// Should lose all KE and rest. Any sustained KE is injection.
	printf("\n=== wedge: dynamic ball into fixed-ball V, COR=%.2f ===\n", cor);
	hop::simulator<T> sim;
	float r=0.4f;
	auto fixedb=[&](hop::vec3<T> p){ auto s=ball(sim,p,r,1.0f); s->set_infinite_mass(); s->set_coefficient_of_gravity(0); s->set_coefficient_of_restitution_override(false); };
	fixedb({-0.5f,0,1}); fixedb({0.5f,0,1});  // two fixed balls forming a valley
	auto d = ball(sim, {0.02f,0,2.2f}, r, cor); // slightly off-center so it isn't perfectly symmetric
	for(int t=1;t<=400;t++){
		sim.update(1.0f/60.0f);
		auto p=d->get_position(); auto v=d->get_velocity();
		float ke=0.5f*(v.x*v.x+v.y*v.y+v.z*v.z);
		if(t<=12||t%40==0) printf("t=%3d pos=(%.4f,%.4f,%.4f) vel=(%.4f,%.4f,%.4f) KE=%.5f act=%d\n",
			t,p.x,p.y,p.z,v.x,v.y,v.z,ke,(int)d->active());
	}
}

static void test_sphere_on_sphere(float offset) {
	printf("\n=== dynamic sphere dropped onto fixed sphere, x-offset=%.2f ===\n", offset);
	hop::simulator<T> sim;
	float r=0.4f;
	auto fx = ball(sim,{0,0,1},r,1.0f); fx->set_infinite_mass(); fx->set_coefficient_of_gravity(0); fx->set_coefficient_of_restitution_override(false);
	auto d = ball(sim,{offset,0,2.5f},r,0.0f);
	for(int t=1;t<=200;t++){
		sim.update(1.0f/60.0f);
		auto p=d->get_position(); auto v=d->get_velocity();
		if(t%10==0) printf("t=%3d pos=(%.3f,%.3f,%.3f) vel=(%.3f,%.3f,%.3f)\n",t,p.x,p.y,p.z,v.x,v.y,v.z);
	}
}

int main(int argc, char**argv) {
	if (argc>1 && !strcmp(argv[1],"sos")) test_sphere_on_sphere(argc>2?atof(argv[2]):0.0f);
	else if (argc>1 && !strcmp(argv[1],"wedge")) test_wedge(argc>2?atof(argv[2]):0.5f);
	else if (argc>1 && !strcmp(argv[1],"gas")) test_gas(argc>2?atoi(argv[2]):6, argc>3?atof(argv[3]):1.0f);
	else if (argc>1 && !strcmp(argv[1],"drop")) test_single_drop();
	else if (argc>1 && !strcmp(argv[1],"stack")) test_stack(argc>2?atoi(argv[2]):4);
	else if (argc>1 && !strcmp(argv[1],"box")) test_box(argc>2?atoi(argv[2]):6, argc>3?atof(argv[3]):0.75f);
	else { test_single_drop(); test_stack(4); }
	return 0;
}
