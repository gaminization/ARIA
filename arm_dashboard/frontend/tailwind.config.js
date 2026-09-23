/** @type {import('tailwindcss').Config} */
export default {
  content: [
    "./index.html",
    "./src/**/*.{js,ts,jsx,tsx}",
  ],
  theme: {
    extend: {
      colors: {
        background: "#0d0f14",
        panel: "#13161e",
        border: "#252a38",
        accent: {
          blue: "#4f8ef7",
          cyan: "#06b6d4",
          violet: "#8b5cf6",
          green: "#4fc9a4",
          amber: "#f7a84f",
          red: "#f76f6f",
        },
      },
      fontFamily: {
        sans: ["Inter", "-apple-system", "BlinkMacSystemFont", "sans-serif"],
        mono: ["JetBrains Mono", "Courier New", "monospace"],
      },
    },
  },
  plugins: [],
}
