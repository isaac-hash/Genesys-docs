import Image from "next/image"
import Link from "next/link"
import { PageRoutes } from "@/lib/pageroutes"   
import "../../app/assets/custom.css"
import { Settings } from "@/types/settings"

export function Footer() {
  return (
    // <footer className="text-foreground flex h-16 w-full flex-wrap items-center justify-center gap-4 border-t px-2 py-3 text-sm sm:justify-between sm:gap-0 sm:px-4 sm:py-0 lg:px-8">
    //   <p className="items-center">
    //     &copy; {new Date().getFullYear()}{" "}
    //     <Link
    //       title={Settings.name}
    //       aria-label={Settings.name}
    //       className="font-semibold"
    //       href={Settings.link}
    //     >
    //       {Settings.name}
    //     </Link>
    //     .
    //   </p>
    //   {Settings.branding !== false && (
    //     <div className="hidden items-center md:block">
    //       <Link
    //         className="font-semibold"
    //         href="https://rubixstudios.com.au"
    //         title="Rubix Studios"
    //         aria-label="Rubix Studios"
    //         target="_blank"
    //       >
    //         <Image
    //           src="/logo.svg"
    //           alt="Rubix Studios logo"
    //           title="Rubix Studios logo"
    //           aria-label="Rubix Studios logo"
    //           priority={false}
    //           width={30}
    //           height={30}
    //         />
    //       </Link>
    //     </div>
    //   )}
    // </footer>
    <>

                <footer>
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
                                <a href="#releases">Releases</a>
                                <a href="#pricing">Future Plans</a>
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
                </footer>
    </>
  )
}
