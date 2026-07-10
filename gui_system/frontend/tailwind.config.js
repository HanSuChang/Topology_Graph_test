/** @type {import('tailwindcss').Config} */
export default {
  content: ["./index.html", "./src/**/*.{ts,tsx}"],
  theme: {
    extend: {
      colors: {
        // Light glass palette. Surfaces are translucent over the body
        // gradient; ink is a soft slate for comfortable reading.
        ink: "#0f172a",
        muted: "#475569",
        line: "rgba(148,163,184,0.30)",
        glass: "rgba(255,255,255,0.55)",
        glassStrong: "rgba(255,255,255,0.75)",
        accent: "#2563eb",
        accentSoft: "#dbeafe",
        leader: "#06b6d4",
        follower1: "#8b5cf6",
        follower2: "#ec4899",
        okSoft: "#dcfce7",
        warnSoft: "#fef3c7",
        errSoft: "#fee2e2",
        // Keep older names as aliases so unconverted bits do not break.
        bg: "#f1f5f9",
        panel: "rgba(255,255,255,0.55)",
      },
      boxShadow: {
        glass: "0 8px 32px rgba(15, 23, 42, 0.08)",
        glassLg: "0 12px 40px rgba(15, 23, 42, 0.10)",
      },
    },
  },
  plugins: [],
};
