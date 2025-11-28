"use client";
import { useEffect } from 'react';
import { Link } from "lib/transition"
import { PageRoutes } from "@/lib/pageroutes"

export default function Page() {
    useEffect(() => {
        // Smooth scroll
        document.querySelectorAll('a[href^="#"]').forEach(anchor => {
            anchor.addEventListener('click', (e) => {
                e.preventDefault();
                const href = anchor.getAttribute('href');
                if (!href) return;
                const target = document.querySelector(href);
                if (target) {
                    target.scrollIntoView({ behavior: 'smooth' });
                }
            });
        });

        // Scroll animations
        const observerOptions = {
            threshold: 0.1,
            rootMargin: '0px 0px -50px 0px'
        };

        const observer = new IntersectionObserver((entries) => {
            entries.forEach(entry => {
                if (entry.isIntersecting) {
                    entry.target.classList.add('animated');
                }
            });
        }, observerOptions);

        document.querySelectorAll('.animate-on-scroll').forEach(el => {
            observer.observe(el);
        });

        // Nav scroll effect
        let lastScroll = 0;
        const handleScroll = () => {
            const nav = document.querySelector('nav');
            if (!nav) return;
            const currentScroll = window.pageYOffset;

            if (currentScroll > 100) {
                nav.classList.add('scrolled');
            } else {
                nav.classList.remove('scrolled');
            }

            lastScroll = currentScroll;
        };
        window.addEventListener('scroll', handleScroll);

        // Stats counter animation
        const statsObserver = new IntersectionObserver((entries) => {
            entries.forEach(entry => {
                if (entry.isIntersecting) {
                    const statNumber = entry.target.querySelector('.stat-number');
                    if (statNumber) {
                        const text = statNumber.textContent || '';
                        const match = text.match(/(\d+\.?\d*)/);
                        if (match) {
                            const target = parseFloat(match[1]);
                            const suffix = text.replace(match[1], '');
                            animateCounter(statNumber as HTMLElement, 0, target, suffix, 2000);
                        }
                        statsObserver.unobserve(entry.target);
                    }
                }
            });
        }, { threshold: 0.5 });

        document.querySelectorAll('.stat-item').forEach(item => {
            statsObserver.observe(item);
        });

        function animateCounter(element: HTMLElement, start: number, end: number, suffix: string, duration: number) {
            const range = end - start;
            const increment = range / (duration / 16);
            let current = start;

            const timer = setInterval(() => {
                current += increment;
                if (current >= end) {
                    element.textContent = end + suffix;
                    clearInterval(timer);
                } else {
                    element.textContent = Math.floor(current) + suffix;
                }
            }, 16);
        }

        return () => {
            window.removeEventListener('scroll', handleScroll);
            observer.disconnect();
            statsObserver.disconnect();
        };
    }, []);

    return (
        <>
            <style jsx global>{`
        :root {
            --bg-primary: #000000;
            --bg-secondary: #0a0a0a;
            --text-primary: #ffffff;
            --text-secondary: #888888;
            --accent: #3b82f6;
            --accent-hover: #2563eb;
            --border: rgba(255, 255, 255, 0.1);
            --card-bg: rgba(255, 255, 255, 0.03);
        }

        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', 'Helvetica Neue', Arial, sans-serif;
            background: var(--bg-primary);
            color: var(--text-primary);
            overflow-x: hidden;
            line-height: 1.6;
            -webkit-font-smoothing: antialiased;
        }

        /* Animated gradient background */
        .gradient-bg {
            position: fixed;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            z-index: 0;
            opacity: 0.4;
        }

        .gradient-orb {
            position: absolute;
            border-radius: 50%;
            filter: blur(120px);
            animation: float 20s infinite ease-in-out;
        }

        .orb-1 {
            width: 600px;
            height: 600px;
            background: radial-gradient(circle, rgba(59, 130, 246, 0.3) 0%, transparent 70%);
            top: -300px;
            right: -200px;
            animation-delay: 0s;
        }

        .orb-2 {
            width: 500px;
            height: 500px;
            background: radial-gradient(circle, rgba(147, 51, 234, 0.2) 0%, transparent 70%);
            bottom: -200px;
            left: -100px;
            animation-delay: 5s;
        }

        .orb-3 {
            width: 400px;
            height: 400px;
            background: radial-gradient(circle, rgba(59, 130, 246, 0.25) 0%, transparent 70%);
            top: 50%;
            left: 50%;
            animation-delay: 10s;
        }

        @keyframes float {

            0%,
            100% {
                transform: translate(0, 0) scale(1);
            }

            33% {
                transform: translate(100px, -50px) scale(1.1);
            }

            66% {
                transform: translate(-50px, 100px) scale(0.9);
            }
        }

        /* Grid overlay */
        .grid-overlay {
            position: fixed;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            background-image:
                linear-gradient(rgba(255, 255, 255, 0.02) 1px, transparent 1px),
                linear-gradient(90deg, rgba(255, 255, 255, 0.02) 1px, transparent 1px);
            background-size: 50px 50px;
            z-index: 1;
            pointer-events: none;
        }

        /* Navigation */
        nav {
            position: fixed;
            top: 0;
            width: 100%;
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 20px 60px;
            background: rgba(0, 0, 0, 0.5);
            backdrop-filter: blur(20px) saturate(180%);
            border-bottom: 1px solid var(--border);
            z-index: 1000;
            transition: all 0.3s ease;
        }

        nav.scrolled {
            padding: 15px 60px;
            background: rgba(0, 0, 0, 0.8);
        }

        .logo {
            font-size: 20px;
            font-weight: 600;
            letter-spacing: -0.5px;
        }

        .nav-links {
            display: flex;
            gap: 40px;
            align-items: center;
        }

        .nav-links a {
            color: var(--text-secondary);
            text-decoration: none;
            font-size: 14px;
            font-weight: 500;
            transition: color 0.2s;
            position: relative;
        }

        .nav-links a:hover {
            color: var(--text-primary);
        }

        .nav-links a.active::after {
            content: '';
            position: absolute;
            bottom: -5px;
            left: 0;
            width: 100%;
            height: 2px;
            background: var(--accent);
        }

        .nav-cta {
            background: var(--text-primary);
            color: var(--bg-primary);
            padding: 8px 20px;
            border-radius: 8px;
            font-size: 14px;
            font-weight: 500;
            border: none;
            cursor: pointer;
            transition: all 0.2s;
        }

        .nav-cta:hover {
            transform: translateY(-1px);
            box-shadow: 0 4px 12px rgba(255, 255, 255, 0.2);
        }

        /* Hero Section */
        .hero {
            min-height: 100vh;
            display: flex;
            align-items: center;
            justify-content: center;
            position: relative;
            z-index: 2;
            padding: 100px 60px 60px;
            background: url('images/bird.gif');
            background-size: cover;
            background-position: center;
            // background-repeat: no-repeat;
            
        }

        .hero-content {
            max-width: 900px;
            text-align: center;
            font-weight: 600;
        }

        .hero-badge {
            display: inline-flex;
            align-items: center;
            gap: 8px;
            padding: 6px 16px;
            background: var(--card-bg);
            border: 1px solid var(--border);
            border-radius: 50px;
            font-size: 13px;
            color: var(--text-secondary);
            margin-bottom: 24px;
            animation: fadeInUp 0.8s ease-out;
        }

        .badge-dot {
            width: 6px;
            height: 6px;
            background: var(--accent);
            border-radius: 50%;
            animation: pulse 2s infinite;
        }

        @keyframes pulse {

            0%,
            100% {
                opacity: 1;
                transform: scale(1);
            }

            50% {
                opacity: 0.5;
                transform: scale(0.8);
            }
        }

        .hero-title {
            font-size: clamp(48px, 8vw, 88px);
            font-weight: 700;
            letter-spacing: -0.04em;
            line-height: 1.1;
            margin-bottom: 24px;
            background: linear-gradient(180deg, #ffffff 0%, #888888 100%);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            background-clip: text;
            animation: fadeInUp 0.8s ease-out 0.2s backwards;
        }

        .hero-subtitle {
            font-size: clamp(18px, 2vw, 24px);
            color: var(--text-secondary);
            line-height: 1.6;
            margin-bottom: 40px;
            animation: fadeInUp 0.8s ease-out 0.4s backwards;
        }

        .hero-cta-group {
            display: flex;
            gap: 16px;
            justify-content: center;
            flex-wrap: wrap;
            animation: fadeInUp 0.8s ease-out 0.6s backwards;
        }

        .btn-primary {
            padding: 14px 32px;
            background: var(--text-primary);
            color: var(--bg-primary);
            border: none;
            border-radius: 10px;
            font-size: 15px;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.2s;
        }

        .btn-primary:hover {
            transform: translateY(-2px);
            box-shadow: 0 8px 24px rgba(255, 255, 255, 0.2);
        }

        .btn-secondary {
            padding: 14px 32px;
            background: transparent;
            color: var(--text-primary);
            border: 1px solid var(--border);
            border-radius: 10px;
            font-size: 15px;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.2s;
        }

        .btn-secondary:hover {
            background: var(--card-bg);
            border-color: rgba(255, 255, 255, 0.2);
        }

        @keyframes fadeInUp {
            from {
                opacity: 0;
                transform: translateY(20px);
            }

            to {
                opacity: 1;
                transform: translateY(0);
            }
        }

        /* Feature Cards */
        .features {
            position: relative;
            z-index: 2;
            padding: 120px 60px;
            max-width: 1400px;
            margin: 0 auto;
        }

        .section-header {
            text-align: center;
            margin-bottom: 80px;
        }

        .section-label {
            display: inline-block;
            padding: 4px 12px;
            background: var(--card-bg);
            border: 1px solid var(--border);
            border-radius: 50px;
            font-size: 13px;
            color: var(--accent);
            margin-bottom: 16px;
            font-weight: 500;
        }

        .section-title {
            font-size: clamp(36px, 5vw, 56px);
            font-weight: 700;
            letter-spacing: -0.03em;
            margin-bottom: 16px;
        }
        .video-title {
            font-size: clamp(26px, 3vw, 36px);
            font-weight: 700;
            letter-spacing: -0.03em;
            margin-bottom: 16px;
        }

        .section-description {
            font-size: 18px;
            color: var(--text-secondary);
            max-width: 600px;
            margin: 0 auto;
        }

        .feature-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(340px, 1fr));
            gap: 24px;
        }

        .feature-card {
            background: var(--card-bg);
            border: 1px solid var(--border);
            border-radius: 16px;
            padding: 32px;
            transition: all 0.3s ease;
            position: relative;
            overflow: hidden;
        }

        .feature-card::before {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            height: 1px;
            background: linear-gradient(90deg, transparent, var(--accent), transparent);
            opacity: 0;
            transition: opacity 0.3s;
        }

        .feature-card:hover {
            border-color: rgba(59, 130, 246, 0.3);
            background: rgba(255, 255, 255, 0.05);
            transform: translateY(-4px);
        }

        .feature-card:hover::before {
            opacity: 1;
        }

        .feature-icon {
            width: 48px;
            height: 48px;
            background: var(--accent);
            border-radius: 12px;
            display: flex;
            align-items: center;
            justify-content: center;
            margin-bottom: 20px;
            font-size: 24px;
        }

        .feature-title {
            font-size: 20px;
            font-weight: 600;
            margin-bottom: 12px;
        }

        .feature-description {
            color: var(--text-secondary);
            font-size: 15px;
            line-height: 1.6;
        }

        /* Video Section */
        .video-section {
            position: relative;
            z-index: 2;
            padding: 120px 60px;
            max-width: 1200px;
            margin: 0 auto;
        }

        .video-container {
            position: relative;
            border-radius: 20px;
            overflow: hidden;
            border: 1px solid var(--border);
            background: var(--card-bg);
            aspect-ratio: 16/9;
        }

        .video-placeholder {
            width: 100%;
            height: 100%;
            background: linear-gradient(135deg, #1a1a1a 0%, #0a0a0a 100%);
            display: flex;
            align-items: center;
            justify-content: center;
            position: relative;
        }

        .play-btn {
            width: 80px;
            height: 80px;
            background: rgba(255, 255, 255, 0.1);
            backdrop-filter: blur(10px);
            border: 2px solid rgba(255, 255, 255, 0.2);
            border-radius: 50%;
            display: flex;
            align-items: center;
            justify-content: center;
            cursor: pointer;
            transition: all 0.3s;
        }

        .play-btn:hover {
            transform: scale(1.1);
            background: rgba(255, 255, 255, 0.15);
        }

        .play-btn::after {
            content: '';
            width: 0;
            height: 0;
            border-left: 20px solid white;
            border-top: 12px solid transparent;
            border-bottom: 12px solid transparent;
            margin-left: 4px;
        }

        /* Stats Section */
        .stats {
            position: relative;
            z-index: 2;
            padding: 120px 60px;
            max-width: 1200px;
            margin: 0 auto;
        }

        .stats-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 60px;
            text-align: center;
        }

        .stat-item {
            opacity: 0;
            animation: fadeInUp 0.8s ease-out forwards;
        }

        .stat-number {
            font-size: 56px;
            font-weight: 700;
            background: linear-gradient(180deg, #ffffff 0%, #888888 100%);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            background-clip: text;
            margin-bottom: 8px;
        }

        .stat-label {
            color: var(--text-secondary);
            font-size: 15px;
        }

        /* CTA Section */
        .cta-section {
            position: relative;
            z-index: 2;
            padding: 120px 60px;
            max-width: 1000px;
            margin: 0 auto;
            text-align: center;
        }

        .cta-card {
            background: var(--card-bg);
            border: 1px solid var(--border);
            border-radius: 24px;
            padding: 80px 60px;
            position: relative;
            overflow: hidden;
        }

        .cta-card::before {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            background: radial-gradient(circle at 50% 0%, rgba(59, 130, 246, 0.1) 0%, transparent 50%);
            pointer-events: none;
        }

        .cta-title {
            font-size: clamp(32px, 4vw, 48px);
            font-weight: 700;
            margin-bottom: 20px;
        }

        .cta-description {
            font-size: 18px;
            color: var(--text-secondary);
            margin-bottom: 40px;
        }

        /* Footer */
        footer {
            position: relative;
            z-index: 2;
            border-top: 1px solid var(--border);
            padding: 60px 60px 40px;
            margin-top: 120px;
        }

        .footer-content {
            max-width: 1400px;
            margin: 0 auto;
            display: grid;
            grid-template-columns: 2fr 1fr 1fr 1fr;
            gap: 60px;
            margin-bottom: 40px;
        }

        .footer-brand {
            font-size: 20px;
            font-weight: 600;
            margin-bottom: 12px;
        }

        .footer-description {
            color: var(--text-secondary);
            font-size: 14px;
            line-height: 1.6;
        }

        .footer-section-title {
            font-size: 14px;
            font-weight: 600;
            margin-bottom: 16px;
        }

        .footer-links {
            display: flex;
            flex-direction: column;
            gap: 12px;
        }

        .footer-links a {
            color: var(--text-secondary);
            text-decoration: none;
            font-size: 14px;
            transition: color 0.2s;
        }

        .footer-links a:hover {
            color: var(--text-primary);
        }

        .footer-bottom {
            max-width: 1400px;
            margin: 0 auto;
            padding-top: 40px;
            border-top: 1px solid var(--border);
            display: flex;
            justify-content: space-between;
            align-items: center;
            color: var(--text-secondary);
            font-size: 14px;
        }

        /* Mobile Menu */
        .mobile-menu-btn {
            display: none;
            background: none;
            border: none;
            color: var(--text-primary);
            font-size: 24px;
            cursor: pointer;
        }

        /* Responsive */
        @media (max-width: 1024px) {
            .footer-content {
                grid-template-columns: 1fr 1fr;
            }
        }

        @media (max-width: 768px) {
            nav {
                padding: 15px 24px;
            }

            .nav-links {
                display: none;
            }

            .mobile-menu-btn {
                display: block;
            }

            .hero {
                padding: 100px 24px 60px;
            }

            .features,
            .video-section,
            .stats,
            .cta-section {
                padding: 80px 24px;
            }

            .feature-grid {
                grid-template-columns: 1fr;
            }

            .stats-grid {
                grid-template-columns: repeat(2, 1fr);
                gap: 40px;
            }

            .cta-card {
                padding: 60px 32px;
            }

            footer {
                padding: 40px 24px 24px;
            }

            .footer-content {
                grid-template-columns: 1fr;
                gap: 40px;
            }

            .footer-bottom {
                flex-direction: column;
                gap: 16px;
                text-align: center;
            }
        }

        /* Scroll animations */
        .animate-on-scroll {
            opacity: 0.01;
            transform: translateY(30px);
            transition: all 0.8s ease-out;
        }

        .animate-on-scroll.animated {
            opacity: 1;
            transform: translateY(0);
        }
            `}</style>
            <div className="gradient-bg">
                <div className="gradient-orb orb-1"></div>
                <div className="gradient-orb orb-2"></div>
                <div className="gradient-orb orb-3"></div>
            </div>

            <div className="grid-overlay"></div>

            {/* <nav>
                <div className="logo">Genesys</div>
                <div className="nav-links">
                    <a href="#features" className="active">Features</a>
                    <Link
                            href={`/docs${PageRoutes[0].href}`}>

                    Documentation
                            </Link>
                    <a href="#community">Community</a>
                    <a href="#about">About</a>
                    <button className="nav-cta">Get Started</button>
                </div>
                <button className="mobile-menu-btn">☰</button>
            </nav> */}

            <section className="hero">
                <div className="hero-content">
                    <div className="hero-badge">
                        <span className="badge-dot"></span>
                        ROS2 Framework v2.0
                    </div>
                    <h1 className="hero-title">Build the future of robotics</h1>
                    <p className="hero-subtitle">
                        Genesys, a modern, powerful framework for developing intelligent robotic systems with unparalleled performance
                        and flexibility
                    </p>
                    <div className="hero-cta-group">
                        <Link
                            href={`/docs${PageRoutes[0].href}`}>
                            <button className="btn-primary">Start Building</button>
                        </Link>
                        <Link
                            href={`/docs${PageRoutes[0].href}`}>
                            <button className="btn-secondary">View Documentation →</button>
                        </Link>
                    </div>
                </div>
            </section>

            <section className="features" id="features">
                <div className="section-header animate-on-scroll">
                    <span className="section-label">FEATURES</span>
                    <h2 className="section-title">Everything you need to succeed</h2>
                    <p className="section-description">
                        Built for developers who demand performance, scalability, and ease of use
                    </p>
                </div>

                <div className="feature-grid">
                    <div className="feature-card animate-on-scroll">
                        <div className="feature-icon">⚡</div>
                        <h3 className="feature-title">Lightning Fast</h3>
                        <p className="feature-description">
                            Optimized performance with minimal latency for real-time robotic applications
                        </p>
                    </div>

                    <div className="feature-card animate-on-scroll">
                        <div className="feature-icon">🔧</div>
                        <h3 className="feature-title">Developer First</h3>
                        <p className="feature-description">
                            Intuitive APIs and comprehensive documentation to get you up and running quickly
                        </p>
                    </div>

                    <div className="feature-card animate-on-scroll">
                        <div className="feature-icon">🛡️</div>
                        <h3 className="feature-title">Production Ready</h3>
                        <p className="feature-description">
                            Battle-tested in real-world scenarios with enterprise-grade reliability
                        </p>
                    </div>

                    <div className="feature-card animate-on-scroll">
                        <div className="feature-icon">🔄</div>
                        <h3 className="feature-title">Seamless Integration</h3>
                        <p className="feature-description">
                            Works perfectly with existing ROS2 ecosystems and third-party tools
                        </p>
                    </div>

                    <div className="feature-card animate-on-scroll">
                        <div className="feature-icon">📊</div>
                        <h3 className="feature-title">Advanced Analytics</h3>
                        <p className="feature-description">
                            Real-time monitoring and debugging tools to optimize your robot&apos;s performance
                        </p>
                    </div>

                    <div className="feature-card animate-on-scroll">
                        <div className="feature-icon">🌐</div>
                        <h3 className="feature-title">Cloud Native</h3>
                        <p className="feature-description">
                            Deploy anywhere from edge devices to cloud infrastructure seamlessly
                        </p>
                    </div>
                </div>
            </section>

            <section className="video-section animate-on-scroll">
                <div className="section-header" style={{ marginBottom: '40px' }}>
                    <span className="section-label">DEMO</span>
                    <h2 className="section-title">See it in action</h2>
                </div>
                    <h2 className="video-title text-center underline">Decorators</h2>
                <div className="video-container mb-4">
                    <div className="video-placeholder">
                        {/* <img src="images/decorators.webm" alt="Demo GIF" style={{ width: '100%', height: '100%', objectFit: 'cover' }} /> */}
                        <video src="images/decorators.webm" autoPlay loop muted style={{ position: 'absolute', top: 0, left: 0, width: '100%', height: '100%', objectFit: 'cover' }}></video>
                        {/* <div className="play-btn"></div> */}
                    </div>
                </div>
                    <h2 className="video-title text-center underline">Macros</h2>
                <div className="video-container">
                    <div className="video-placeholder">
                        {/* <img src="images/macros.webm" alt="Demo GIF" style={{ width: '100%', height: '100%', objectFit: 'cover' }} /> */}
                        <video src="images/macros.webm" autoPlay loop muted style={{ position: 'absolute', top: 0, left: 0, width: '100%', height: '100%', objectFit: 'cover' }}></video>
                        {/* <div className="play-btn"></div> */}
                    </div>
                </div>
            </section>

            {/* <section className="stats">
                <div className="stats-grid">
                    <div className="stat-item">
                        <div className="stat-number">50K+</div>
                        <div className="stat-label">Active Developers</div>
                    </div>
                    <div className="stat-item" style={{ animationDelay: '0.1s' }}>
                        <div className="stat-number">200+</div>
                        <div className="stat-label">Companies Trust Us</div>
                    </div>
                    <div className="stat-item" style={{ animationDelay: '0.2s' }}>
                        <div className="stat-number">99.9%</div>
                        <div className="stat-label">Uptime Guaranteed</div>
                    </div>
                    <div className="stat-item" style={{ animationDelay: '0.3s' }}>
                        <div className="stat-number">10M+</div>
                        <div className="stat-label">API Calls Daily</div>
                    </div>
                </div>
            </section> */}

            <section className="cta-section">
                <div className="cta-card animate-on-scroll">
                    <h2 className="cta-title">Ready to build something amazing?</h2>
                    <p className="cta-description">
                        Join thousands of developers building the future of robotics with Genesys
                    </p>
                    <div className="hero-cta-group">
                        <Link
                            href={`/docs${PageRoutes[0].href}`}>
                            <button className="btn-primary">Get Started for Free</button>
                        </Link>
                        <a href="https://www.linkedin.com/in/isaac-chukwudulue-439b01321/" target="_blank" rel="noopener noreferrer">
                            <button className="btn-secondary">Talk to The Team</button>
                        </a>
                    </div>
                </div>
            </section>

            {/* <footer>
                <div className="footer-content">
                    <div>
                        <div className="footer-brand">Genesys</div>
                        <p className="footer-description">
                            The modern ROS2 framework for building intelligent robotic systems with unprecedented performance
                            and flexibility.
                        </p>
                    </div>
                    <div>
                        <h4 className="footer-section-title">Product</h4>
                        <div className="footer-links">
                            <a href="#features">Features</a>
                            <Link
                            href={`/docs${PageRoutes[0].href}`}
                            >
                                Documentation
                            </Link>
                            <a href="#pricing">Pricing</a>
                            <a href="#releases">Releases</a>
                        </div>
                    </div>
                    <div>
                        <h4 className="footer-section-title">Company</h4>
                        <div className="footer-links">
                            <a href="#about">About</a>
                            <a href="#blog">Blog</a>
                            <a href="#careers">Careers</a>
                            <a href="#contact">Contact</a>
                        </div>
                    </div>
                    <div>
                        <h4 className="footer-section-title">Resources</h4>
                        <div className="footer-links">
                            <a href="#community">Community</a>
                            <a href="#tutorials">Tutorials</a>
                            <a href="#support">Support</a>
                            <a href="#status">Status</a>
                        </div>
                    </div>
                </div>
                <div className="footer-bottom">
                    <div>© 2025 Genesys. All rights reserved.</div>
                    <div>
                        <a href="#privacy"
                            style={{ color: 'var(--text-secondary)', textDecoration: 'none', marginRight: '24px' }}>Privacy</a>
                        <a href="#terms" style={{ color: 'var(--text-secondary)', textDecoration: 'none' }}>Terms</a>
                    </div>
                </div>
            </footer> */}
        </>
    );
}