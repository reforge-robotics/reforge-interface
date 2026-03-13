(() => {
  const WIDGET_TAG = "reforge-robotics-widget";
  const DEFAULT_MANAGE_URL = "https://app.reforgerobotics.com";
  const DEFAULT_POLL_INTERVAL_MS = 1500;
  const DEFAULT_POLL_TIMEOUT_MS = 5 * 60 * 1000;

  function getCurrentScript() {
    return document.currentScript;
  }

  function normalizeOptional(value) {
    const normalized = (value || "").trim();
    if (!normalized) return "";
    // Treat template placeholders as unset values.
    if (normalized === "REPLACE_WITH_ROBOT_IP") return "";
    if (normalized === "REPLACE_WITH_LOCAL_IP") return "";
    if (normalized === "REPLACE_WITH_SDK_TOKEN") return "";
    if (normalized === "REPLACE_WITH_REFORGE_API_TOKEN") return "";
    if (normalized === "REPLACE_WITH_REFORGE_ROBOT_ID") return "";
    return normalized;
  }

  function getDataConfig(script) {
    return {
      baseUrl: normalizeOptional(script?.dataset?.baseUrl || script?.dataset?.reforgeBaseUrl),
      manageUrl: normalizeOptional(script?.dataset?.manageUrl || script?.dataset?.reforgeManageUrl),
      mountSelector: normalizeOptional(script?.dataset?.mount || script?.dataset?.reforgeMount) || "body",
      robotIp: normalizeOptional(script?.dataset?.robotIp || script?.dataset?.reforgeRobotIp),
      localIp: normalizeOptional(script?.dataset?.localIp || script?.dataset?.reforgeLocalIp),
      sdkToken: normalizeOptional(script?.dataset?.sdkToken || script?.dataset?.reforgeSdkToken),
      reforgeRobotId: normalizeOptional(script?.dataset?.reforgeRobotId),
      identifyApiToken: normalizeOptional(script?.dataset?.reforgeIdentifyApiToken),
      freq: normalizeOptional(script?.dataset?.freq || script?.dataset?.reforgeFreq),
      pollIntervalMs: Number(script?.dataset?.pollIntervalMs || script?.dataset?.reforgePollIntervalMs) || DEFAULT_POLL_INTERVAL_MS,
      pollTimeoutMs: Number(script?.dataset?.pollTimeoutMs || script?.dataset?.reforgePollTimeoutMs) || DEFAULT_POLL_TIMEOUT_MS
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

    if (config.robotIp && !widget.hasAttribute("robot-ip")) {
      widget.setAttribute("robot-ip", config.robotIp);
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
    widget.connecting = !!state.connecting;
    widget.calibrating = !!state.calibrating;
  }

  function buildRunPayload(widget, config) {
    const robotId = normalizeOptional(widget.getAttribute("robot-id"));
    const robotIp = normalizeOptional(widget.getAttribute("robot-ip")) || config.robotIp;
    return {
      robotIp,
      localIp: config.localIp,
      sdkToken: config.sdkToken,
      reforgeRobotId: robotId || config.reforgeRobotId,
      identifyApiToken: config.identifyApiToken,
      freq: config.freq,
      robotId
    };
  }

  async function pollConnectUntilComplete(widget, config, endpoints) {
    const start = Date.now();

    while (Date.now() - start < config.pollTimeoutMs) {
      const status = await fetchJson(endpoints.connectStatus, { method: "GET" });
      const connecting = !!status.connecting;
      widget.connecting = connecting;

      if (status.status === "succeeded") {
        return status;
      }
      if (status.status === "failed") {
        const errorMessage = status.error || status.stderr || "Connect test failed.";
        throw new Error(errorMessage);
      }
      await new Promise((resolve) => setTimeout(resolve, config.pollIntervalMs));
    }

    throw new Error("Connect status polling timed out");
  }

  async function pollCalibrationUntilComplete(widget, config, endpoints) {
    const start = Date.now();

    while (Date.now() - start < config.pollTimeoutMs) {
      const status = await fetchJson(endpoints.calibrateStatus, { method: "GET" });
      const calibrating = !!status.calibrating;
      widget.connecting = !!status.connecting;
      widget.calibrating = calibrating;

      if (status.status === "succeeded") {
        return status;
      }
      if (status.status === "failed") {
        const errorMessage = status.error || status.stderr || "Calibration failed.";
        if (status.phase === "connect_test") {
          throw new Error("Robot connection unsuccessful");
        }
        throw new Error(errorMessage);
      }
      await new Promise((resolve) => setTimeout(resolve, config.pollIntervalMs));
    }

    throw new Error("Calibration status polling timed out");
  }

  async function initialize() {
    const currentScript = getCurrentScript();
    const config = getDataConfig(currentScript);

    if (!config.baseUrl) {
      console.error("Reforge auto widget: missing data-base-url");
      return;
    }

    await loadCoreWidgetScript(currentScript);

    const widget = mountWidget(config);
    const endpoints = {
      state: joinUrl(config.baseUrl, "/api/reforge/state"),
      vibration: joinUrl(config.baseUrl, "/api/reforge/vibration"),
      robotId: joinUrl(config.baseUrl, "/api/reforge/robot-id"),
      connectStart: joinUrl(config.baseUrl, "/api/reforge/connect/start"),
      connectStatus: joinUrl(config.baseUrl, "/api/reforge/connect/status"),
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
        const robotId = event.detail?.id || "";
        config.reforgeRobotId = robotId || config.reforgeRobotId;
        if (currentScript && robotId) {
          currentScript.dataset.reforgeRobotId = robotId;
        }
        await fetchJson(endpoints.robotId, {
          method: "POST",
          body: JSON.stringify({ id: robotId })
        });
        const state = await fetchJson(endpoints.state, { method: "GET" });
        applyState(widget, state);
      } catch (error) {
        console.error("Reforge auto widget: robot-id update failed", error);
      } finally {
        widget.busy = false;
      }
    });

    widget.addEventListener("reforge:updateRobotIp", (event) => {
      const robotIp = normalizeOptional(event.detail?.ip || "");
      if (!robotIp) return;
      config.robotIp = robotIp;
      if (currentScript) {
        currentScript.dataset.robotIp = robotIp;
      }
    });

    widget.addEventListener("reforge:calibrateRobot", async () => {
      try {
        const payload = buildRunPayload(widget, config);
        widget.calibrating = true;
        widget.connecting = true;
        widget.connectionStatus = "running";
        widget.connectionMessage = "Running connect test...";
        await fetchJson(endpoints.calibrateStart, { method: "POST", body: JSON.stringify(payload) });
        await pollCalibrationUntilComplete(widget, config, endpoints);
        widget.connectionStatus = "success";
        widget.connectionMessage = "Robot is connected";
      } catch (error) {
        console.error("Reforge auto widget: calibration failed", error);
        if (String(error?.message || "").toLowerCase().includes("connection")) {
          widget.connectionStatus = "error";
          widget.connectionMessage = "Robot connection unsuccessful";
        }
      } finally {
        widget.connecting = false;
        widget.calibrating = false;
      }
    });

    widget.addEventListener("reforge:connectTest", async () => {
      try {
        const payload = buildRunPayload(widget, config);
        widget.connecting = true;
        widget.connectionStatus = "running";
        widget.connectionMessage = "Running connect test...";
        await fetchJson(endpoints.connectStart, { method: "POST", body: JSON.stringify(payload) });
        await pollConnectUntilComplete(widget, config, endpoints);
        widget.connectionStatus = "success";
        widget.connectionMessage = "Robot is connected";
      } catch (error) {
        console.error("Reforge auto widget: connect test failed", error);
        widget.connectionStatus = "error";
        widget.connectionMessage = "Robot connection unsuccessful";
      } finally {
        widget.connecting = false;
      }
    });
  }

  initialize().catch((error) => {
    console.error("Reforge auto widget: initialization failed", error);
  });
})();
