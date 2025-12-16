---
title: Physical AI & Humanoid Robotics Book
---

<!-- Modern responsive homepage with hero section, featured modules, and CTAs -->

<div class="hero hero--primary" style={{background: 'linear-gradient(135deg, #1a2a6c, #2a4d69, #4b86b4)', padding: '6rem 0', textAlign: 'center', color: 'white', marginBottom: '2rem'}}>
  <div class="container">
    <h1 class="hero__title" style={{fontSize: '3rem', fontWeight: 'bold', marginBottom: '1rem', textShadow: '2px 2px 4px rgba(0,0,0,0.3)'}}>Bridge Digital AI with Humanoid Robotics</h1>
    <p class="hero__subtitle" style={{fontSize: '1.5rem', marginBottom: '2rem', opacity: 0.9}}>Explore Physical AI concepts, Humanoid Robotics, and AI-Robot interactions</p>
    <div class="button-group">
      <a class="button button--secondary button--lg" href="#modules" style={{background: 'white', color: '#1a2a6c', border: 'none', padding: '1rem 2rem', fontSize: '1.2rem', borderRadius: '50px', textDecoration: 'none', transition: 'all 0.3s ease', margin: '0.5rem', boxShadow: '0 4px 15px rgba(0,0,0,0.2)'}}>Explore Modules</a>
    </div>
  </div>
</div>

<div id="modules" style={{padding: '4rem 0', background: '#f8f9fa'}}>
  <div class="container">
    <h2 style={{textAlign: 'center', fontSize: '2.5rem', marginBottom: '3rem', color: '#2a4d69'}}>Featured Modules</h2>

    <div style={{display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(300px, 1fr))', gap: '2rem', marginBottom: '3rem'}}>

      <!-- Module 1 Card -->
      <div class="card" style={{background: 'white', borderRadius: '12px', padding: '2rem', boxShadow: '0 8px 25px rgba(0,0,0,0.1)', transition: 'all 0.3s ease', border: '1px solid #e9ecef', cursor: 'pointer'}} onMouseOver="this.style.transform='scale(1.03)'; this.style.boxShadow='0 12px 35px rgba(0,0,0,0.15)';" onMouseOut="this.style.transform='scale(1)'; this.style.boxShadow='0 8px 25px rgba(0,0,0,0.1)';">
        <div style={{textAlign: 'center', marginBottom: '1.5rem'}}>
          <div style={{width: '80px', height: '80px', background: 'linear-gradient(135deg, #4b86b4, #2a4d69)', borderRadius: '50%', margin: '0 auto 1rem', display: 'flex', alignItems: 'center', justifyContent: 'center', color: 'white', fontSize: '2rem'}}>🤖</div>
          <h3 class="card__title" style={{fontSize: '1.5rem', fontWeight: 'bold', color: '#2a4d69', margin: '0 0 1rem 0'}}>Robotic Nervous System (ROS 2)</h3>
        </div>
        <div class="card__body">
          <ul style={{listStyle: 'none', padding: 0, margin: 0}}>
            <li style={{marginBottom: '0.5rem'}}><a href="#module-1-ros-2-intro" style={{color: '#1a2a6c', textDecoration: 'none', transition: 'color 0.2s ease'}}>• Introduction to ROS 2</a></li>
            <li style={{marginBottom: '0.5rem'}}><a href="#module-1-ros-2-nodes" style={{color: '#1a2a6c', textDecoration: 'none', transition: 'color 0.2s ease'}}>• Nodes, Topics, Services</a></li>
            <li style={{marginBottom: '0.5rem'}}><a href="#module-1-ros-2-python" style={{color: '#1a2a6c', textDecoration: 'none', transition: 'color 0.2s ease'}}>• Python Agent Bridging</a></li>
            <li style={{marginBottom: '0.5rem'}}><a href="#module-1-ros-2-urdf" style={{color: '#1a2a6c', textDecoration: 'none', transition: 'color 0.2s ease'}}>• URDF for Humanoids</a></li>
          </ul>
        </div>
      </div>

      <!-- Module 2 Card -->
      <div class="card" style={{background: 'white', borderRadius: '12px', padding: '2rem', boxShadow: '0 8px 25px rgba(0,0,0,0.1)', transition: 'all 0.3s ease', border: '1px solid #e9ecef', cursor: 'pointer'}} onMouseOver="this.style.transform='scale(1.03)'; this.style.boxShadow='0 12px 35px rgba(0,0,0,0.15)';" onMouseOut="this.style.transform='scale(1)'; this.style.boxShadow='0 8px 25px rgba(0,0,0,0.1)';">
        <div style={{textAlign: 'center', marginBottom: '1.5rem'}}>
          <div style={{width: '80px', height: '80px', background: 'linear-gradient(135deg, #6a11cb, #2575fc)', borderRadius: '50%', margin: '0 auto 1rem', display: 'flex', alignItems: 'center', justifyContent: 'center', color: 'white', fontSize: '2rem'}}>🔄</div>
          <h3 class="card__title" style={{fontSize: '1.5rem', fontWeight: 'bold', color: '#2a4d69', margin: '0 0 1rem 0'}}>Digital Twin (Gazebo & Unity)</h3>
        </div>
        <div class="card__body">
          <ul style={{listStyle: 'none', padding: 0, margin: 0}}>
            <li style={{marginBottom: '0.5rem'}}><a href="#module-2-digital-twins" style={{color: '#1a2a6c', textDecoration: 'none', transition: 'color 0.2s ease'}}>• Digital Twins Fundamentals</a></li>
            <li style={{marginBottom: '0.5rem'}}><a href="#module-2-gazebo-physics" style={{color: '#1a2a6c', textDecoration: 'none', transition: 'color 0.2s ease'}}>• Gazebo Physics Simulation</a></li>
            <li style={{marginBottom: '0.5rem'}}><a href="#module-2-unity-visualization" style={{color: '#1a2a6c', textDecoration: 'none', transition: 'color 0.2s ease'}}>• Unity Visualization</a></li>
            <li style={{marginBottom: '0.5rem'}}><a href="#module-2-sensor-simulation" style={{color: '#1a2a6c', textDecoration: 'none', transition: 'color 0.2s ease'}}>• Sensor Simulation</a></li>
          </ul>
        </div>
      </div>

      <!-- Module 3 Card -->
      <div class="card" style={{background: 'white', borderRadius: '12px', padding: '2rem', boxShadow: '0 8px 25px rgba(0,0,0,0.1)', transition: 'all 0.3s ease', border: '1px solid #e9ecef', cursor: 'pointer'}} onMouseOver="this.style.transform='scale(1.03)'; this.style.boxShadow='0 12px 35px rgba(0,0,0,0.15)';" onMouseOut="this.style.transform='scale(1)'; this.style.boxShadow='0 8px 25px rgba(0,0,0,0.1)';">
        <div style={{textAlign: 'center', marginBottom: '1.5rem'}}>
          <div style={{width: '80px', height: '80px', background: 'linear-gradient(135deg, #ff416c, #ff4b2b)', borderRadius: '50%', margin: '0 auto 1rem', display: 'flex', alignItems: 'center', justifyContent: 'center', color: 'white', fontSize: '2rem'}}>🧠</div>
          <h3 class="card__title" style={{fontSize: '1.5rem', fontWeight: 'bold', color: '#2a4d69', margin: '0 0 1rem 0'}}>AI-Robot Brain (NVIDIA Isaac™)</h3>
        </div>
        <div class="card__body">
          <ul style={{listStyle: 'none', padding: 0, margin: 0}}>
            <li style={{marginBottom: '0.5rem'}}><a href="#module-3-ai-brain" style={{color: '#1a2a6c', textDecoration: 'none', transition: 'color 0.2s ease'}}>• AI Brain Fundamentals</a></li>
            <li style={{marginBottom: '0.5rem'}}><a href="#module-3-isaac-sim" style={{color: '#1a2a6c', textDecoration: 'none', transition: 'color 0.2s ease'}}>• Isaac Sim Environment</a></li>
            <li style={{marginBottom: '0.5rem'}}><a href="#module-3-perception-navigation" style={{color: '#1a2a6c', textDecoration: 'none', transition: 'color 0.2s ease'}}>• Perception & Navigation</a></li>
            <li style={{marginBottom: '0.5rem'}}><a href="#module-3-training-perception" style={{color: '#1a2a6c', textDecoration: 'none', transition: 'color 0.2s ease'}}>• Training & Perception</a></li>
          </ul>
        </div>
      </div>

    </div>
  </div>
</div>

<div style={{padding: '4rem 0', background: 'white'}}>
  <div class="container">
    <h2 style={{textAlign: 'center', fontSize: '2.5rem', marginBottom: '3rem', color: '#2a4d69'}}>Module Overview</h2>

    <div style={{display: 'flex', justifyContent: 'center', gap: '3rem', flexWrap: 'wrap', marginBottom: '3rem'}}>

      <!-- Module 1 Overview -->
      <div style={{textAlign: 'center', maxWidth: '300px'}}>
        <div style={{width: '100px', height: '100px', background: 'linear-gradient(135deg, #4b86b4, #2a4d69)', borderRadius: '50%', margin: '0 auto 1rem', display: 'flex', alignItems: 'center', justifyContent: 'center', color: 'white', fontSize: '3rem'}}>🔗</div>
        <h3 style={{color: '#2a4d69', margin: '1rem 0 0.5rem 0'}}>Module 1</h3>
        <p style={{color: '#6c757d'}}>ROS 2 middleware and communication patterns for robot systems</p>
      </div>

      <!-- Module 2 Overview -->
      <div style={{textAlign: 'center', maxWidth: '300px'}}>
        <div style={{width: '100px', height: '100px', background: 'linear-gradient(135deg, #6a11cb, #2575fc)', borderRadius: '50%', margin: '0 auto 1rem', display: 'flex', alignItems: 'center', justifyContent: 'center', color: 'white', fontSize: '3rem'}}>🌐</div>
        <h3 style={{color: '#2a4d69', margin: '1rem 0 0.5rem 0'}}>Module 2</h3>
        <p style={{color: '#6c757d'}}>Digital twin simulation with Gazebo and Unity environments</p>
      </div>

      <!-- Module 3 Overview -->
      <div style={{textAlign: 'center', maxWidth: '300px'}}>
        <div style={{width: '100px', height: '100px', background: 'linear-gradient(135deg, #ff416c, #ff4b2b)', borderRadius: '50%', margin: '0 auto 1rem', display: 'flex', alignItems: 'center', justifyContent: 'center', color: 'white', fontSize: '3rem'}}>💡</div>
        <h3 style={{color: '#2a4d69', margin: '1rem 0 0.5rem 0'}}>Module 3</h3>
        <p style={{color: '#6c757d'}}>AI brain systems with perception, navigation, and training</p>
      </div>

      <!-- Module 4 Overview -->
      <div style={{textAlign: 'center', maxWidth: '300px'}}>
        <div style={{width: '100px', height: '100px', background: 'linear-gradient(135deg, #11998e, #38ef7d)', borderRadius: '50%', margin: '0 auto 1rem', display: 'flex', alignItems: 'center', justifyContent: 'center', color: 'white', fontSize: '3rem'}}>👁️</div>
        <h3 style={{color: '#2a4d69', margin: '1rem 0 0.5rem 0'}}>Module 4</h3>
        <p style={{color: '#6c757d'}}>Vision-Language-Action systems for robot interaction</p>
      </div>

    </div>
  </div>
</div>

<div style={{padding: '4rem 0', background: 'linear-gradient(135deg, #6a11cb, #2575fc)', textAlign: 'center', color: 'white'}}>
  <div class="container">
    <h2 style={{fontSize: '2.5rem', marginBottom: '2rem', textShadow: '2px 2px 4px rgba(0,0,0,0.3)'}}>Ready to Start Your Journey?</h2>
    <p style={{fontSize: '1.2rem', marginBottom: '2rem', opacity: 0.9, maxWidth: '600px', marginLeft: 'auto', marginRight: 'auto'}}>Begin exploring the intersection of AI and robotics with our comprehensive modules.</p>
    <div class="button-group">
      <a class="button button--secondary button--lg" href="/docs/intro" style={{background: 'white', color: '#6a11cb', border: 'none', padding: '1rem 2rem', fontSize: '1.2rem', borderRadius: '50px', textDecoration: 'none', transition: 'all 0.3s ease', margin: '0.5rem', boxShadow: '0 4px 15px rgba(0,0,0,0.2)', display: 'inline-flex', alignItems: 'center', gap: '0.5rem'}}>
        <span>📖</span> Start Reading
      </a>
      <a class="button button--secondary button--lg" href="/docs/module-1-ros2" style={{background: 'transparent', color: 'white', border: '2px solid white', padding: '1rem 2rem', fontSize: '1.2rem', borderRadius: '50px', textDecoration: 'none', transition: 'all 0.3s ease', margin: '0.5rem', display: 'inline-flex', alignItems: 'center', gap: '0.5rem'}}>
        <span>🤖</span> Ask Chatbot
      </a>
    </div>
  </div>
</div>