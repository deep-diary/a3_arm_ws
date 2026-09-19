/* Mermaid init — 全站共用 */
mermaid.initialize({
  startOnLoad: true,
  securityLevel: "loose",
  theme: "base",
  themeVariables: {
    fontFamily: '"Segoe UI","Microsoft YaHei",sans-serif',
    fontSize: "13px",
    primaryColor: "#e8f0fa",
    primaryTextColor: "#1f3b63",
    primaryBorderColor: "#2e74b5",
    lineColor: "#5a6a7a",
    secondaryColor: "#f4f8fc",
    tertiaryColor: "#ffffff",
    clusterBkg: "#fbfdff",
    clusterBorder: "#c8d4e0",
    titleColor: "#1f3b63",
    edgeLabelBackground: "#ffffff",
  },
  flowchart: {
    htmlLabels: true,
    curve: "basis",
    padding: 14,
    nodeSpacing: 28,
    rankSpacing: 62,
    diagramPadding: 8,
  },
  sequence: {
    actorMargin: 28,
    messageMargin: 36,
    mirrorActors: false,
  },
});
