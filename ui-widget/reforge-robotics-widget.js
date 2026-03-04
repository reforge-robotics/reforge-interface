(() => {
  const template = document.createElement("template");
  template.innerHTML = `
    <style>
      @import url('https://fonts.googleapis.com/css2?family=Host+Grotesk:ital,wght@0,300..800;1,300..800&display=swap');

      :host {
        --rw-bg: #0f0f0f; /* {~.~} Edit background color if desired */
        --rw-panel: #1b1b1b; /* {~.~} Edit panel color if desired */
        --rw-panel-light: #efe8dd;
        --rw-text: #f4f2ed; /* {~.~} Edit text color if desired */
        --rw-muted: #cfc7bc; /* {~.~} Edit muted text color if desired */
        --rw-primary: #5a8396;
        --rw-primary-dark: #3c6375;
        --rw-secondary: #e5dfd4;
        --rw-border: #2d2d2d;
        --rw-input-bg: #d9d1c6;
        --rw-input-text: #4a4844;
        --rw-radius: 10px;
        display: inline-block;
        font-family: "Host Grotesk", "Helvetica Neue", Arial, sans-serif;
      }

      .widget {
        background: var(--rw-panel);
        color: var(--rw-text);
        padding: 18px;
        border-radius: 0px;
        min-width: 260px;
        max-width: 410px;
        width: 100%;
        font-family: inherit;
      }

      input,
      button {
        font-family: inherit;
      }

      .poweredBy {
        font-size: 11px;
        letter-spacing: 0.08em;
        text-transform: uppercase;
        color: var(--rw-muted);
        margin-bottom: 10px;
      }

      .statusRow {
        display: flex;
        align-items: center;
        gap: 10px;
        margin-bottom: 14px;
      }

      .statusLabel {
        font-size: 13px;
        letter-spacing: 0.02em;
        color: var(--rw-muted);
      }

      .toggle {
        position: relative;
        width: 48px;
        height: 26px;
        border-radius: 999px;
        border: 1px solid var(--rw-border);
        background: #dcd6cb;
        cursor: pointer;
        outline: none;
        padding: 0;
      }

      .toggleKnob {
        position: absolute;
        top: 3px;
        left: 4px;
        width: 18px;
        height: 18px;
        border-radius: 50%;
        background: #2d2d2d;
        display: grid;
        place-items: center;
        font-size: 12px;
        color: #f4f2ed;
        transition: transform 0.2s ease, background 0.2s ease;
      }

      .toggle[aria-checked="true"] {
        background: #c8deec;
      }

      .toggle[aria-checked="true"] .toggleKnob {
        transform: translateX(20px);
        background: #5a8396;
      }

      .section {
        margin-top: 14px;
      }

      .label {
        display: block;
        font-size: 12px;
        letter-spacing: 0.04em;
        text-transform: uppercase;
        color: var(--rw-muted);
        margin-bottom: 6px;
      }

      .idRow {
        display: flex;
        gap: 8px;
        align-items: center;
      }

      .idInput {
        flex: 1;
        border: 1px solid #b8afa4;
        background: var(--rw-input-bg);
        color: var(--rw-input-text);
        padding: 8px 10px;
        font-size: 14px;
      }

      .idInput:disabled {
        cursor: not-allowed;
        opacity: 0.75;
      }

      .btn {
        border: none;
        font-size: 13px;
        padding: 8px 12px;
        cursor: pointer;
      }

      .btn:disabled {
        cursor: not-allowed;
        opacity: 0.7;
      }

      .btnPrimary {
        background: var(--rw-primary);
        color: #f4f2ed;
      }

      .btnPrimary:hover {
        background: var(--rw-primary-dark);
      }

      .btnSecondary {
        background: var(--rw-secondary);
        color: #1d1b18;
      }

      .btnGhost {
        background: transparent;
        border: 1px solid #c4bbb0;
        color: #cfc7bc;
      }

      .actions {
        margin-top: 12px;
        display: flex;
        gap: 10px;
        flex-wrap: wrap;
      }

      .fieldMessage {
        margin-top: 8px;
        font-size: 12px;
        color: #f0c997;
      }

      .callout {
        background: #fff0b4;
        color: #2f2b24;
        padding: 12px;
        font-size: 14px;
        line-height: 1.3;
        margin-bottom: 10px;
      }

      .hidden {
        display: none;
      }

      .busy {
        opacity: 0.65;
        pointer-events: none;
      }
    </style>
    <div class="widget" part="widget">
      <div class="poweredBy">Powered by Reforge</div>
      <div class="statusRow">
        <button class="toggle" type="button" role="switch" aria-checked="false" aria-label="Toggle vibration control">
          <span class="toggleKnob" aria-hidden="true">&#10003;</span>
        </button>
        <div>
          <div class="statusLabel statusText">Vibration control OFF</div>
        </div>
      </div>

      <div class="section unregisteredOnly">
        <div class="callout"></div>
      </div>

      <div class="section">
        <span class="label">Robot ID</span>
        <div class="idRow">
          <input class="idInput" type="text" placeholder="Enter robot ID" />
          <button class="btn btnGhost updateId" type="button">Edit</button>
        </div>
        <div class="fieldMessage hidden"></div>
      </div>

      <div class="section registeredOnly">
        <div class="actions">
          <button class="btn btnPrimary calibrate" type="button">Calibrate robot</button>
          <button class="btn btnSecondary manage" type="button">Manage on Reforge App</button>
        </div>
      </div>

      <div class="section unregisteredOnly">
        <div class="actions">
          <button class="btn btnPrimary register" type="button">Register robot on Reforge App</button>
        </div>
      </div>
    </div>
  `;

  /**
   * Reforge Robotics widget web component.
   * Public API: attributes `robot-id`, `vibration-enabled`, `mode`, `message`,
   * `manage-url`, `busy`, `calibrating` and the CustomEvents in docs.
   * Side effects: dispatches CustomEvents and may open a new window for portal
   * actions unless the event is canceled.
   */
  class ReforgeRoboticsWidget extends HTMLElement {
    static get observedAttributes() {
      return [
        "robot-id",
        "vibration-enabled",
        "mode",
        "message",
        "manage-url",
        "busy",
        "calibrating"
      ];
    }

    /**
     * Initializes shadow DOM and caches element handles.
     */
    constructor() {
      super();
      this.attachShadow({ mode: "open" });
      this.shadowRoot.appendChild(template.content.cloneNode(true));
      // Shadow DOM element handles keyed by role.
      this._elements = {
        root: this.shadowRoot.querySelector(".widget"),
        toggle: this.shadowRoot.querySelector(".toggle"),
        toggleKnob: this.shadowRoot.querySelector(".toggleKnob"),
        statusText: this.shadowRoot.querySelector(".statusText"),
        idInput: this.shadowRoot.querySelector(".idInput"),
        updateId: this.shadowRoot.querySelector(".updateId"),
        fieldMessage: this.shadowRoot.querySelector(".fieldMessage"),
        calibrate: this.shadowRoot.querySelector(".calibrate"),
        manage: this.shadowRoot.querySelector(".manage"),
        register: this.shadowRoot.querySelector(".register"),
        registeredOnly: this.shadowRoot.querySelectorAll(".registeredOnly"),
        unregisteredOnly: this.shadowRoot.querySelectorAll(".unregisteredOnly"),
        callout: this.shadowRoot.querySelector(".callout")
      };
      this._bound = false; // Guard to prevent double-binding event handlers.
      this._calibrateTimer = null; // Demo-only timeout id.
    }

    /**
     * Cleanup hook for any outstanding timers.
     */
    disconnectedCallback() {
      clearTimeout(this._calibrateTimer);
    }

    /**
     * Lifecycle hook invoked when the element is added to the DOM.
     * Side effects: upgrades pre-set properties, binds handlers, and renders.
     */
    connectedCallback() {
      this._upgradeProperty("robotId");
      this._upgradeProperty("vibrationEnabled");
      this._upgradeProperty("mode");
      this._upgradeProperty("message");
      this._upgradeProperty("manageUrl");
      this._upgradeProperty("busy");
      this._upgradeProperty("calibrating");
      this._bind();
      this._render();
    }

    /**
     * Lifecycle hook invoked when observed attributes change.
     * Side effects: triggers a re-render.
     */
    attributeChangedCallback() {
      this._render();
    }

    /**
     * Ensures pre-upgrade properties are re-applied as attributes.
     * @param {string} prop - camelCase property name.
     */
    _upgradeProperty(prop) {
      if (Object.prototype.hasOwnProperty.call(this, prop)) {
        const value = this[prop];
        delete this[prop];
        this[prop] = value;
      }
    }

    /**
     * Binds UI event handlers once per element instance.
     * Side effects: dispatches CustomEvents for host integration.
     */
    _bind() {
      if (this._bound) return;
      this._bound = true;

      // Toggle vibration state and emit event for host integration.
      this._elements.toggle.addEventListener("click", () => {
        const next = !this.vibrationEnabled;
        this.vibrationEnabled = next;
        this.dispatchEvent(
          new CustomEvent("reforge:toggleVibration", {
            detail: { enabled: next },
            bubbles: true,
            composed: true
          })
        );
      });

      // Edit/save robot id with validation; emits update event on success.
      this._elements.updateId.addEventListener("click", () => {
        if (this._isEditing) {
          const nextId = this._elements.idInput.value.trim();
          if (!this._isValidRobotId(nextId)) {
            this._setFieldMessage("Robot ID must be a valid UUID.");
            return;
          }
          this._setFieldMessage("");
          const previousId = this.robotId;
          this.robotId = nextId;
          if (this._currentMode() === "unregistered") {
            this.mode = "registered";
          }
          this._isEditing = false;
          this.dispatchEvent(
            new CustomEvent("reforge:updateRobotId", {
              detail: { id: nextId, previousId },
              bubbles: true,
              composed: true
            })
          );
          this._render();
          return;
        }
        this._isEditing = true;
        this._setFieldMessage("");
        this._render();
        this._elements.idInput.focus();
        this._elements.idInput.select();
      });

      // Emit calibration event and lock the button until host clears state.
      this._elements.calibrate.addEventListener("click", () => {
        if (this.calibrating) return;
        this.calibrating = true;
        this._updateCalibrateButton();
        this._render();
        this.dispatchEvent(
          new CustomEvent("reforge:calibrateRobot", {
            detail: {},
            bubbles: true,
            composed: true
          })
        );
      });

      // Emit portal open event; default is opening a new window.
      this._elements.manage.addEventListener("click", () => {
        const event = new CustomEvent("reforge:openManagePortal", {
          detail: { url: this.manageUrl },
          bubbles: true,
          composed: true,
          cancelable: true
        });
        this.dispatchEvent(event);
        if (!event.defaultPrevented) {
          window.open(this.manageUrl, "_blank", "noopener,noreferrer");
        }
      });

      // Emit registration event; default is opening a new window.
      this._elements.register.addEventListener("click", () => {
        const event = new CustomEvent("reforge:registerRobot", {
          detail: { url: this.manageUrl },
          bubbles: true,
          composed: true,
          cancelable: true
        });
        this.dispatchEvent(event);
        if (!event.defaultPrevented) {
          window.open(this.manageUrl, "_blank", "noopener,noreferrer");
        }
      });
    }

    /**
     * Renders UI state based on attributes and internal flags.
     * Side effects: updates shadow DOM text, attributes, and visibility.
     */
    _render() {
      const mode = this._currentMode();
      const busy = this.busy;
      const message = this.message ||
        "For best results, register and calibrate your robot to update the base vibration model.";
      const hasRobotId = !!this.robotId;
      const isEditing = this._isEditing || !hasRobotId;
      const placeholder = mode === "unregistered"
        ? "Register this robot to get an ID"
        : "Enter robot ID";

      this._elements.registeredOnly.forEach((section) => {
        section.classList.toggle("hidden", mode !== "registered");
      });
      this._elements.unregisteredOnly.forEach((section) => {
        section.classList.toggle("hidden", mode !== "unregistered");
      });
      this._elements.callout.textContent = message;
      this._elements.idInput.value = mode === "unregistered" ? "" : (this.robotId || "");
      this._elements.idInput.placeholder = placeholder;
      this._elements.idInput.disabled = !isEditing;
      this._elements.updateId.textContent = isEditing ? "Save" : "Edit";
      this._updateCalibrateButton();
      this._elements.toggle.setAttribute("aria-checked", String(!!this.vibrationEnabled));
      this._elements.toggleKnob.textContent = this.vibrationEnabled ? "\u2713" : "\u00D7";
      this._elements.statusText.textContent = this.vibrationEnabled
        ? "Vibration control ON"
        : "Vibration control OFF";
      this._elements.root.classList.toggle("busy", busy);
    }

    /**
     * Validates robot ID format.
     * @param {string} value - candidate robot ID.
     * @returns {boolean} True when value matches UUID v1-v5 format.
     */
    _isValidRobotId(value) {
      return (
        typeof value === "string" &&
        /^[0-9a-f]{8}-[0-9a-f]{4}-[1-5][0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/i.test(value)
      );
    }

    /**
     * Applies calibrating state to the Calibrate button.
     * Side effects: toggles button disabled state and label.
     */
    _updateCalibrateButton() {
      if (!this._elements.calibrate) return;
      const isCalibrating = !!this.calibrating;
      this._elements.calibrate.disabled = isCalibrating;
      this._elements.calibrate.textContent = isCalibrating
        ? "Calibrating..."
        : "Calibrate robot";
    }

    /**
     * Sets or clears the inline field message.
     * @param {string} message - message text (empty to clear).
     */
    _setFieldMessage(message) {
      if (!this._elements.fieldMessage) return;
      this._elements.fieldMessage.textContent = message;
      this._elements.fieldMessage.classList.toggle("hidden", !message);
    }

    /**
     * Resolves display mode from explicit attribute or robot-id presence.
     * @returns {"registered" | "unregistered"} Current display mode.
     */
    _currentMode() {
      const modeAttr = this.getAttribute("mode");
      if (modeAttr === "registered" || modeAttr === "unregistered") {
        return modeAttr;
      }
      return this.robotId ? "registered" : "unregistered";
    }

    /**
     * Parses boolean attributes with common HTML truthy values.
     * @param {string} name - attribute name.
     * @returns {boolean} Parsed boolean value.
     */
    _parseBooleanAttr(name) {
      const value = this.getAttribute(name);
      if (value === null) return false;
      if (value === "" || value === "true") return true;
      return value === "1";
    }

    /**
     * @returns {string} Current robot id value.
     */
    get robotId() {
      return this.getAttribute("robot-id") || "";
    }

    /**
     * @param {string} value - robot id to set, empty clears attribute.
     */
    set robotId(value) {
      if (value === null || value === undefined || value === "") {
        this.removeAttribute("robot-id");
      } else {
        this.setAttribute("robot-id", value);
      }
    }

    /**
     * @returns {boolean} Whether vibration control is enabled.
     */
    get vibrationEnabled() {
      return this._parseBooleanAttr("vibration-enabled");
    }

    /**
     * @param {boolean} value - new vibration state.
     */
    set vibrationEnabled(value) {
      if (value) {
        this.setAttribute("vibration-enabled", "true");
      } else {
        this.removeAttribute("vibration-enabled");
      }
    }

    /**
     * @returns {string} Explicit mode attribute, if set.
     */
    get mode() {
      return this.getAttribute("mode") || "";
    }

    /**
     * @param {string} value - "registered" or "unregistered" (empty clears).
     */
    set mode(value) {
      if (!value) {
        this.removeAttribute("mode");
      } else {
        this.setAttribute("mode", value);
      }
    }

    /**
     * @returns {string} Callout message text.
     */
    get message() {
      return this.getAttribute("message") || "";
    }

    /**
     * @param {string} value - callout message to display (empty clears).
     */
    set message(value) {
      if (!value) {
        this.removeAttribute("message");
      } else {
        this.setAttribute("message", value);
      }
    }

    /**
     * @returns {string} Portal URL for manage/register actions.
     */
    get manageUrl() {
      return this.getAttribute("manage-url") || "https://app.reforgerobotics.com";
    }

    /**
     * @param {string} value - portal URL (empty clears).
     */
    set manageUrl(value) {
      if (!value) {
        this.removeAttribute("manage-url");
      } else {
        this.setAttribute("manage-url", value);
      }
    }

    /**
     * @returns {boolean} Whether the widget should appear busy/disabled.
     */
    get busy() {
      return this._parseBooleanAttr("busy");
    }

    /**
     * @param {boolean} value - busy state.
     */
    set busy(value) {
      if (value) {
        this.setAttribute("busy", "true");
      } else {
        this.removeAttribute("busy");
      }
    }

    /**
     * @returns {boolean} Whether calibration is in progress.
     */
    get calibrating() {
      return this._parseBooleanAttr("calibrating");
    }

    /**
     * @param {boolean} value - calibration state.
     */
    set calibrating(value) {
      if (value) {
        this.setAttribute("calibrating", "true");
      } else {
        this.removeAttribute("calibrating");
      }
    }
  }

  if (!customElements.get("reforge-robotics-widget")) {
    customElements.define("reforge-robotics-widget", ReforgeRoboticsWidget);
  }
})();
