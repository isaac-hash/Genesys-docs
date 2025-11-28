import { PageRoutes } from "@/lib/pageroutes"
import { NavItem } from "@/types/navigation"

export const Navigations: NavItem[] = [
  {
    title: "Docs",
    href: `/docs${PageRoutes[0].href}`,
  },
  // {
  //   title: "Rubix",
  //   href: "https://rubixstudios.com.au",
  //   external: true,
  // },
]

export const GitHubLink = {
  href: "https://github.com/isaac-hash/Genesys",
}
