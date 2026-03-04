(() => {
  const WIDGET_TAG = "reforge-robotics-widget";
  const DEFAULT_MANAGE_URL = "https://app.reforgerobotics.com";
  const DEFAULT_POLL_INTERVAL_MS = 1500;
  const DEFAULT_POLL_TIMEOUT_MS = 5 * 60 * 1000;

  function getCurrentScript() {
    return document.currentScript;
  }

  function getDataConfig(script) {
    return {
      baseUrl: (script?.dataset?.reforgeBaseUrl || "").trim(),
      manageUrl: (script?.dataset?.reforgeManageUrl || "").trim(),
      mountSelector: (script?.dataset?.reforgeMount || "body").trim(),
      pollIntervalMs: Number(script?.dataset?.reforgePollIntervalMs) || DEFAULT_POLL_INTERVAL_MS,
      pollTimeoutMs: Number(script?.dataset?.reforgePollTimeoutMs) || DEFAULT_POLL_TIMEOUT_MS
    };
  }

  function joinUrl(baseUrl, path) {
    const base = baseUrl.replace(/\/+$/, "");
    const suffix = path.replace(/^\/+/, "");
    return `${base}/${suffix}`;
  }

  async function loadCoreWidgetScript(scriptEl) {
    if (customElements.get(WIDGET_TAG)) {
      return;
    }

    const autoScriptUrl = new URL(scriptEl.src, window.location.href);
    const coreScriptUrl = new URL("./reforge-robotics-widget.js", autoScriptUrl).toString();

    await new Promise((resolve, reject) => {
      const existing = document.querySelector(`script[src="${coreScriptUrl}"]`);
      if (existing) {
        existing.addEventListener("load", () => resolve(), { once: true });
        existing.addEventListener("error", () => reject(new Error("Failed to load widget script")), {
          once: true
        });
        return;
      }

      const script = document.createElement("script");
      script.src = coreScriptUrl;
      script.async = true;
      script.addEventListener("load", () => resolve(), { once: true });
      script.addEventListener("error", () => reject(new Error("Failed to load widget script")), {
        once: true
      });
      document.head.appendChild(script);
    });
  }

  async function fetchJson(url, options) {
    const response = await fetch(url, {
      headers: { "Content-Type": "application/json" },
      ...options
    });

    if (!response.ok) {
      const body = await response.text().catch(() => "");
      throw new Error(`Request failed (${response.status}): ${url}${body ? ` :: ${body}` : ""}`);
    }

    const contentType = response.headers.get("content-type") || "";
    if (!contentType.includes("application/json")) {
      return {};
    }

    return response.json();
  }

  function mountWidget(config) {
    const mountRoot = document.querySelector(config.mountSelector) || document.body;
    let widget = mountRoot.querySelector(WIDGET_TAG);

    if (!widget) {
      widget = document.createElement(WIDGET_TAG);
      mountRoot.appendChild(widget);
    }

    if (config.manageUrl) {
      widget.setAttribute("manage-url", config.manageUrl);
    } else if (!widget.hasAttribute("manage-url")) {
      widget.setAttribute("manage-url", DEFAULT_MANAGE_URL);
    }

    return widget;
  }

  function applyState(widget, state) {
    const robotId = typeof state.robotId === "string" ? state.robotId : "";
    const vibrationEnabled = !!state.vibrationEnabled;
    const registered = !!state.registered || !!robotId;

    if (robotId) {
      widget.setAttribute("robot-id", robotId);
    } else {
      widget.removeAttribute("robot-id");
    }

    if (vibrationEnabled) {
      widget.setAttribute("vibration-enabled", "true");
    } else {
      widget.removeAttribute("vibration-enabled");
    }

    widget.setAttribute("mode", registered ? "registered" : "unregistered");
  }

  async function pollCalibrationUntilComplete(widget, config, endpoints) {
    const start = Date.now();

    while (Date.now() - start < config.pollTimeoutMs) {
      const status = await fetchJson(endpoints.calibrateStatus, { method: "GET" });
      const calibrating = !!status.calibrating;
      widget.calibrating = calibrating;
      if (!calibrating) {
        return;
      }
      await new Promise((resolve) => setTimeout(resolve, config.pollIntervalMs));
    }

    throw new Error("Calibration status polling timed out");
  }

  async function initialize() {
    const currentScript = getCurrentScript();
    const config = getDataConfig(currentScript);

    if (!config.baseUrl) {
      console.error("Reforge auto widget: missing data-reforge-base-url");
      return;
    }

    await loadCoreWidgetScript(currentScript);

    const widget = mountWidget(config);
    const endpoints = {
      state: joinUrl(config.baseUrl, "/api/reforge/state"),
      vibration: joinUrl(config.baseUrl, "/api/reforge/vibration"),
      robotId: joinUrl(config.baseUrl, "/api/reforge/robot-id"),
      calibrateStart: joinUrl(config.baseUrl, "/api/reforge/calibrate/start"),
      calibrateStatus: joinUrl(config.baseUrl, "/api/reforge/calibrate/status")
    };

    try {
      widget.busy = true;
      const state = await fetchJson(endpoints.state, { method: "GET" });
      applyState(widget, state);
    } catch (error) {
      console.error("Reforge auto widget: failed to fetch initial state", error);
    } finally {
      widget.busy = false;
    }

    if (widget.dataset.reforgeAutoBound === "true") {
      return;
    }
    widget.dataset.reforgeAutoBound = "true";

    widget.addEventListener("reforge:toggleVibration", async (event) => {
      try {
        widget.busy = true;
        await fetchJson(endpoints.vibration, {
          method: "POST",
          body: JSON.stringify({ enabled: !!event.detail?.enabled })
        });
      } catch (error) {
        console.error("Reforge auto widget: toggle failed", error);
        const state = await fetchJson(endpoints.state, { method: "GET" }).catch(() => null);
        if (state) {
          applyState(widget, state);
        }
      } finally {
        widget.busy = false;
      }
    });

    widget.addEventListener("reforge:updateRobotId", async (event) => {
      try {
        widget.busy = true;
        await fetchJson(endpoints.robotId, {
          method: "POST",
          body: JSON.stringify({ id: event.detail?.id || "" })
        });
        const state = await fetchJson(endpoints.state, { method: "GET" });
        applyState(widget, state);
      } catch (error) {
        console.error("Reforge auto widget: robot-id update failed", error);
      } finally {
        widget.busy = false;
      }
    });

    widget.addEventListener("reforge:calibrateRobot", async () => {
      try {
        widget.calibrating = true;
        await fetchJson(endpoints.calibrateStart, { method: "POST", body: JSON.stringify({}) });
        await pollCalibrationUntilComplete(widget, config, endpoints);
      } catch (error) {
        console.error("Reforge auto widget: calibration failed", error);
      } finally {
        widget.calibrating = false;
      }
    });
  }

  initialize().catch((error) => {
    console.error("Reforge auto widget: initialization failed", error);
  });
})();