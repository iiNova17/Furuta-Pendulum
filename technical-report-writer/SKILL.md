---
name: "technical-report-writer"
description: "Write, guide, review, or edit engineering lab reports, design reports, concept documents and step-by-step procedures in Nedal's own writing style: reproducible, visual, well-designed, and submission-ready at all times."
---

# Technical Report Writer

Goal: a report another engineer can reproduce or check without asking the author anything. It is written in Nedal's voice, uses visuals wherever they help understanding, looks clean and professional, and is ready to present or submit whenever it is handed over.

Adapted from: blader/humanizer, K-Dense scientific-writing, anthropics/doc-coauthoring, atalariq/typst-lab-report, obra/the-elements-of-style. Voice and layout derived from Nedal's own reports: converging-nozzle thrust derivation (LaTeX), MEP212 drag-device design report (Word), and the PLUME Technical Concept Document (LaTeX; layout reference only, see section 4).

## Core rule: two channels, never mixed

**Channel 1: the report.** Only final, presentable content written for its real reader (instructor, reviewer, sponsor, team). Every version handed over must be submittable as is.

Never put any of these in the report:
- placeholders or markers: `[TODO]`, `TBD`, `XX`, `???`, `[insert ...]`, "Your abstract.", empty headings, highlighted gaps, empty cells to fill later
- notes about missing information: "data not available", "to be measured"
- references to the user or the conversation: "the user provided", "as requested", "based on the data given"
- drafting comments, alternatives ("or alternatively..."), questions
- internal provenance tags like [Measured] / [Assumed]; state sources the way reports normally do (citations, captions, method descriptions)
- AI disclaimers or chat phrasing

**Channel 2: the chat reply.** Everything addressed to the user: questions, gaps, assumptions needing approval, requirement-checklist status, review findings, fresh-reader test results, figure sources needing confirmation, suggestions.

## Asking the user

- Before drafting, list every fact the report needs, check it against the provided material, and ask for everything missing in one short batch (multiple-choice question tools when available; otherwise a numbered list), each with a few words on why it is needed.
- If a new gap appears mid-draft, stop and ask. Never fill it with a guess.
- Never invent measurements, results, sample sizes, statistics, citations, datasheet values, quotes, or plot data.
- Never introduce an assumption without approval. Once approved, write it in the report the way Nedal does (section 3).
- If the user cannot provide something, keep the report presentable: narrow the claim, restructure, or omit it, and tell the user in chat what changed and why.
- Ask about ambiguous constraints ("does the cover page count toward the 10 pages?") instead of guessing.

## 0. Pick type and mode

**Type:** A. Lab / project report | B. Procedure / runbook | C. Design report / trade study | D. Concept / proposal document.

**Mode** (ask if unclear; for graded coursework, mention that Guide exists):
- **Guide** - in chat: outline, requirement checklist, per-section guiding questions and required facts. The user writes; then Review.
- **Draft** - write the full report from the user's facts and data.
- **Review** - audit the user's draft; findings in chat; a corrected report, if requested, keeps every claim and the user's voice.
- **Edit** - change an existing document (e.g. a .tex or .docx that already exists). Keep its section structure, front matter, figures, tables, equations, numbers and layout exactly as they are unless the user asks for a change. Do not add a cover page, abstract or new sections on your own; if the document seems to need them, ask in chat. Content that was already there (including figures and their captions) is left untouched; any problem you notice in it goes in the chat note, not into the document. Rewriting prose into Nedal's voice (section 3) and fixing technical defects that do not change content (e.g. the font-encoding fix in section 6) are allowed. Before and after editing, extract every equation and every number from the document and confirm they are identical unless a change was requested.

## 1. Before writing

1. Read every provided file: assignment sheet, rubric, template, datasheets, data logs, code, prior reports. Build a requirement checklist (page/word limit, required sections, figure/table rules, citation style, formatting, what counts toward page count). Keep it in chat.
2. Keep the user's template, cover page and section order unless told otherwise.
3. Collect facts: part numbers, pin maps, versions, measured values, test conditions, instruments, sample sizes, number of trials. Ask for gaps.
4. Decide the output format (section 6) and the visuals plan (section 5) before drafting.

## 2. Structure by type

### A. Lab / project report
Cover page (user's template) -> Abstract -> Introduction/Objective -> Theory or Pre-design Analysis -> Experimental Setup(s) -> Analysis / Design -> Results (actual vs expected) -> Discussion -> Conclusion -> References -> Appendices. Follow the assignment's required sections over this default. For a new document, the cover page, abstract and an introduction that states the objective are required unless the user says otherwise. When editing an existing document, keep its existing structure instead (Edit mode).
- Abstract: one paragraph narrating the whole work in order: objective, method (experiments, models), design choice, iterations, outcome against the requirement.
- Introduction: one or two sentences of physical background, then the objective stated with its numerical target, then one sentence on the approach.
- Experimental setup: alternatives considered and why the chosen method won; instrument models with datasheet accuracy; how data was acquired and processed; outlier handling stated explicitly; numbered limitations list.
- Results: data with units, conditions, number of trials; accepted range derived from repeated trials; no interpretation yet.
- Discussion: actual vs expected, percent deviation with the formula shown, numbered contributing factors (specific, not generic "human error"), what was done about them.
- Conclusion: one short paragraph: goal, method, outcome against the target, main limitation.

### B. Procedure / runbook
- Numbered steps, one action each, imperative mood.
- State-changing steps end with **Expected:** the exact observable result (e.g. `3.30 V +/- 0.05 V`).
- **If not:** for steps that commonly fail.
- Commands, paths, register values, config lines in copy-pasteable code blocks.
- Preconditions before the step that needs them; safety warning immediately before the hazardous step. Never instruct energizing motors, actuators or mains circuits without a preceding explicit safety-check step.
- Phases with checkpoints; troubleshooting table: symptom | likely cause | check | fix.

### C. Design report / trade study
Problem and numbered measurable requirements -> options considered -> weighted trade table with justified scores -> selected design (described as proposed until validated) -> analysis -> iterations -> risks -> verification plan.

### D. Concept / proposal document (PLUME layout)
Problem understanding and technical motivation (with the governing physics) -> conceptual workflow figure -> proposed implementation per subsystem -> hardware block diagram and layered software architecture figures -> requirements table (ID, description, type, priority) -> task allocation and RACI -> feature matrix (must / should / stretch) -> Gantt timeline and milestone table -> constraints / scaling discussion with comparison table.

### All types
- Units on every quantity, with a space between number and unit (`5 kg`, `0.0393 kg`).
- Equations numbered; every symbol defined right after the equation that introduces it; dimensional check.
- Tables for comparisons, specifications, requirements and trade studies.
- Datasheet and textbook values cited with source (and page/table where possible).

## 3. Nedal's voice (default for all prose)

Write the way Nedal writes his own reports: an engineer reasoning through the problem in order, physics first, every step justified, honest about limitations. Plain vocabulary, no hype.

The target is a student engineer explaining his reasoning to the reader, not a terse journal paper. Before drafting or editing any prose, reread the reference passages in section 3A and the do/don't table in section 3B. Imitate the passages; the rules below only describe them.

**Flow of an analysis section**
1. Start from the governing physics: free-body diagram, then Newton's second law, conservation of mass/energy, or the ideal gas law. Typical opener: "By applying Newton's second law to the vehicle in the x-direction (see Figure 1):"
2. Expand the equation step by step. Introduce each new term in a "where ..." sentence directly after the equation.
3. Connect steps with short transitions: "Expanding this equation further yields:", "Substituting it back into equation 18 yields:", "Rewriting it as:", "integrating both sides with the initial conditions ...:", "solving for ...:". In derivations, use the therefore sign (\therefore) and because sign (\because) at the start of equation lines.
4. After each important result, state what it means physically in one sentence: "Equation 24 shows the tank's air mass as a function of time, which is a decaying exponential." or "which can be intuitively understood as ...".
5. End a design calculation with the result in bold: "Thus, the initial design of the drag system should have **A_d C_dd = 7235 mm^2**." Then check it against the validity condition of any correlation used.

**Assumptions and values**
- State the assumption with its physical reason in the same sentence: "Since the air discharge process happens very fast, the flow will be assumed to be adiabatic."
- When a property has a range, give the range and its source, then pick conservatively and say so: "a worst-case value of 0.08 will be taken"; "an initial design target not to exceed".
- Admit an assumption's weakness and explain why it is still useful: "While that assumption is not very accurate, since ..., it can provide initial values for ..., which can then be modified iteratively and verified experimentally."
- Justify method choices against alternatives with numbers, e.g. polynomial degree chosen by comparing residual sum of squares and R^2 across degrees, with a short parenthetical caveat.

**Sentence habits to keep**
- Purpose openers: "This paper focuses on ... in order to ...", "The objective of this work is to ...", "To achieve the aim of this work, ...", "it is desired to ...", "it is required to ...".
- Clause openers: "Since ...,", "By ...ing ...,", "Thus,", "Therefore,", "Consequently,", "Furthermore,".
- Refer to the project as "this work". Use "we" / "our" naturally in analysis ("our system", "we can see") and passive voice for procedures and experiments ("an experiment was conducted", "will be taken"), unless the template or course requires one voice.
- Backward cross-references everywhere: "(see Figure 4)", "(see inequality 10)", "previously obtained in section 3.3", "(previously defined at the end of section 3.1)".
- Short parenthetical clarifications for non-obvious quantities: "(1 means a perfect fit)", "(the sum of the squares of the difference between all datapoints and their projection on the curve)".
- Check the result against an earlier prediction: "... peaking at a specific position as previously predicted in section 3, which is a good indication thus far for the followed methodology."
- Limitations and error sources as numbered lists with lettered sub-items, each naming a concrete mechanism.
- Long, explanatory sentences are fine; one idea per sentence, though.
- Figure captions are full descriptive phrases: "A scatter plot of the obtained experimental displacement data against time"; for adapted figures, append the source in brackets.

**Fix, do not imitate** (habits from drafts that should not carry over)
- Typos, inconsistent capitalization of "figure"/"equation", missing spaces before units, duplicated equation labels, wrong units (check every unit), "Reynold's" (write "Reynolds").
- Run-on sentences that join two separate ideas with semicolons; split them.
- Unfinished sections or template leftovers.
- Mixed British and American spelling; use American spelling consistently (center, modeled, analyze).

## 3A. Reference passages (Nedal's own writing, lightly corrected)

These are the target voice. Match their sentence length, transitions, reasoning order and amount of explanation.

**Motivation before a derivation (nozzle report)**
> Since the system's dynamic model (equation 3) consists of design parameters ($C_{rr}$, $C_d$, $m$, $A$), constants, and the vehicle's velocity and its derivative, it is desired to find the thrust force as a parametric function of time, where $F_t = F_t(t)$, so that we can model the system as an ODE in the time domain and further iterate over the model's parameters to achieve an optimal design.

**Condition, assumption with its reason (nozzle report)**
> The inequality in equation 6 is known as the choking condition. Therefore, the flow in our system is choked (sonic) as long as the tank's stagnation pressure remains greater than or equal to approximately 1.92 bar (inequality 10). Since the air discharge process happens very fast, the flow will be assumed to be adiabatic with negligible heat transfer ($Q = 0$).

**Derivation chain with meaning after the result (nozzle report)**
> Equation 18 shows the transient behavior of the nozzle's thrust, since the mass flow rate is directly proportional to the mass of the air inside the tank, which is decreasing with time. Since the mass of the air inside the tank is decreasing at the rate of the mass flowing out of the nozzle, the change in the tank's air mass can be expressed as follows:
> $$\frac{dm_{at}}{dt} = -\dot m \qquad (19)$$
> Substituting $\dot m$ from equation 18:
> $$-\frac{dm_{at}}{dt} = 211.313\, m_{at}\, A^* c \qquad (20)$$
> $$\therefore\; -\frac{dm_{at}}{m_{at}} = 211.313\, A^* c\, dt \qquad (21)$$
> Integrating both sides with the initial condition $m_{at} = 0.0393$ kg at $t = 0$ (see equation 5), and solving for $m_{at}(t)$:
> $$\therefore\; m_{at}(t) = 0.0393\, e^{-211.313 A^* c\, t} \qquad (24)$$
> Equation 24 shows the tank's air mass as a function of time, which is a decaying exponential.

**Design values chosen conservatively (nozzle report)**
> Setting an initial vehicle mass of 5 kg, which is an initial design target not to exceed. For TPU wheels, since TPU closely resembles polyurethane (PU), which has a rolling resistance coefficient ranging from 0.04 to 0.08, a worst-case value of 0.08 will be taken.

**Method choice against alternatives (drag-device report)**
> To accurately track the motion of the toy car, multiple experimental setups were considered, such as video tracking, attaching the car to a motion encoder, and setting multiple velocity sensors along the path. Eventually, an experimental setup using an ultrasonic sensor was built. The video tracking method had many drawbacks due to the inaccurate reading of the car's position and the large amount of manual labor required, while the motion encoder method required a mechanism that would slow down the car. The ultrasonic method, however, required much less manual labor, did not interfere with the car's motion, and was much more accurate.

**Justifying a choice with numbers (drag-device report)**
> For the fitting of the curve, a 5th degree polynomial was chosen, which resulted in a residual sum of squares of 26.77 (the sum of the squares of the differences between all data points and their projections on the generated curve) and a coefficient of determination of 0.999052 (1 means a perfect fit). A 4th degree fit had a residual sum of squares of almost 40, while higher-order fits offered very little improvement over the 5th degree (for example, the 10th degree had a residual sum of squares of 19.675). Thus, to avoid unnecessarily complex expressions, the 5th degree was deemed suitable for this work.

**Admitting an assumption's weakness (drag-device report)**
> While that assumption is not very accurate, since adding a drag device is expected to shift the position of the maximum velocity, our objective of reaching 21 cm/s at 30.8 cm is not guaranteed to be the new maximum velocity, but is expected to happen somewhere near it. However, despite the inaccuracy of this assumption, it can provide us with initial values of $A_d C_{dd}$ for the design of the drag device, which can then be modified iteratively and verified experimentally to achieve the objective of this work.

**Bold result (drag-device report)**
> Thus, the initial design of the drag system should have $\mathbf{A_d C_{dd} = 7235\ mm^2}$.

**Limitations list and acceptance range (drag-device report)**
> Limitations of the aforementioned experimental setup are:
> 1. Its limited range (1 m maximum).
> 2. The minimum reading interval of 50 ms; although it appears low, it is not that low for a toy car that finishes its trip in almost 2 seconds.
> 3. The sensor's resolution of about 3 mm, as previously mentioned.
> 4. The setup is prone to human errors, such as:
>    a. the difficulty of setting the pulling distance (backward stroke) of the toy car at exactly 5 cm;
>    b. the difficulty of controlling the initial orientation of the car relative to the track;
>    c. giving the car a slight push when releasing it, resulting in a non-zero initial velocity.
>
> Due to these limitations, the experiment was conducted multiple times, with results ranging from 39 cm/s to 43.3 cm/s for $V_{max}$ and from 29 cm to 32 cm for $X_{max}$. Thus, any after-design results achieving half of $V_{max}$ within that range of positions would be considered successful.

## 3B. Do / don't (common drifts away from Nedal's voice)

| Do (Nedal) | Don't (terse journal / textbook style) |
|---|---|
| State why a step is taken before taking it: "it is desired to ... so that ...", "To find ..., ..." | Start a step with no motivation |
| Narrate derivation steps as actions: "Differentiating equation 3 with respect to time gives:", "By substituting ...", "Substituting it back into equation 18 yields:" | Instructions to the reader: "Differentiate ...", "Define ...", "Let ...", "First, ...", "Next, ..." ("Let" is fine only for naming a symbol) |
| \therefore on the result line of each multi-step derivation, \because for the fact used | Derivations with no \therefore / \because |
| A sentence on the physical meaning after each important result; check it against earlier predictions | Moving on to the next equation, or a bare one-line fact |
| "we" / "our system" in analysis; passive voice for experiments | Fully impersonal prose throughout |
| Key results and final design values in bold | No emphasis on results |
| Cross-references written out: "equation 18", "(see equation 18)", "Figure 4", "(see Figure 4)", "section 3.1", "previously obtained in section 3.3" | "Eq. (18)", "Eqs. (16) and (18)", "Fig. 4", "Sec. 3" |
| Numbered subsections (3.2, 3.2.1) for topics; numbered lists with lettered sub-items for limitations and error sources | Bold run-in paragraph labels ("Shape of the arm friction.", LaTeX `\paragraph{}`) |
| Short paragraphs, one idea each | Long paragraphs packing several facts together |
| Parenthetical clarifications for non-obvious quantities | Undefined jargon or unexplained statistics |
| "expanding and simplifying (using MATLAB's symbolic toolbox)" | "solving" for an algebraic expansion |

## 4. Avoid: AI patterns

The PLUME concept document was AI-assisted: use it as the layout reference, not the voice. Words like "leveraging", "innovative", "indispensable" and "robust" in it are exactly what to avoid.

- *Staging:* "not X but Y" / "It's not just X, it's Y", dramatic one-line closers, aphorisms, run-ups ("Here's the thing:"), rebutting claims nobody made.
- *Rhythm by rule:* reflexive triplets, repeated sentence openings, em dashes as the default connector, stacked qualifiers or hedges.
- *Inflation:* delve, crucial, pivotal, seamless, leverage, utilize, innovative, indispensable, cutting-edge, landscape, realm, tapestry, intricate, meticulous, showcase, underscore, foster, streamline, empower, elevate, holistic, synergy, paradigm, testament, ever-evolving; non-technical robust/comprehensive/navigate/optimal (allowed as technical terms: robust control, optimal control, optimal design found by iteration); significance inflation; sales language; trailing -ing clauses that fake analysis.
- *Borrowed authority:* "studies show", "experts agree"; cite the source or delete.
- *Formatting by rule:* decorative bold, a heading on every paragraph, bullets where prose is expected, emoji.
- *Chat leftovers:* "Certainly", "I hope this helps", cutoff disclaimers, a first sentence repeating the heading, "This section will discuss ...", restating "In conclusion" summaries.

## 5. Visuals

Use a visual wherever it makes the report easier to read or understand. Default: every analysis has its free-body or schematic diagram, every experiment has a setup figure, every dataset has a plot, every architecture has a block diagram, every comparison has a table.

**Sources, in order of preference**
1. **User-provided** figures, photos, CAD renders, plots and data. Use these first; ask in chat if a needed one is missing (e.g. a photo of the setup).
2. **Generated** by Claude:
   - plots from the user's real data or from the report's own equations (matplotlib or pgfplots), e.g. raw data with the fitted curve, model vs measurement, parameter sweeps of a derived ODE, accepted-range bands;
   - diagrams drawn in TikZ, Mermaid, Graphviz or schemdraw: free-body diagrams, setup schematics, block diagrams, layered architectures, flowcharts, workflows, Gantt charts, circuit diagrams.
   Never plot invented or "illustrative" data as if measured. A sketch of expected trend shape must be labeled as such in its caption.
3. **Sourced online**: datasheet figures, textbook tables or correlations, standard diagrams. Prefer redrawing them in the report's style. Always cite the source in the caption ("[Adapted from Cengel, Fluid Mechanics: Fundamentals and Applications]") and in References. List every online-sourced figure in chat so the user can confirm it is acceptable for submission. Never use photos of people.

**Every figure**
- Numbered, captioned with a descriptive phrase (what, under what conditions), referenced in the text before it appears.
- Axes labeled with quantity and unit, e.g. "Displacement [cm]", "Time [s]". Legends only when there are several series.
- Readable without color: distinct markers and line styles as well as colors.
- Vector output (PDF/SVG) for plots and diagrams; raster images at 300 dpi or higher.
- Sized to the text width and legible at final print size (fonts in figures close to body text size).

## 6. Document design

The document must look clean and professional, consistent from cover to references.

**Format**
- Follow the user's template when given (e.g. a university Word cover page in Times New Roman): build the .docx with that template and its fonts.
- Otherwise default to LaTeX compiled to PDF (matching Nedal's nozzle and PLUME documents), delivering both the .pdf and the .tex source with figure files.

**LaTeX defaults**
- `article` (or `report` for long theses), A4, `geometry` margins about 2.5 cm, 11-12 pt Latin Modern / Computer Modern.
- Always include `\usepackage[T1]{fontenc}`, `\usepackage{lmodern}` and `\usepackage{microtype}` (and `\usepackage{cmap}` with pdfLaTeX) so the PDF text is searchable and copyable: ligatures (fi, ff) and dashes must survive copy-paste. After compiling, run `pdftotext` on the PDF and check for broken words like "gure" or "dierent".
- Packages: `amsmath`, `siunitx` (units and number formatting), `booktabs` (tables, no vertical rules), `graphicx`, `float`, `caption` (bold figure/table labels), `subcaption`, `xcolor`, `titlesec` (colored section headings), `fancyhdr` (running header: short title left, team/author right; page number in footer), `hyperref` with colored links, `tikz` and `pgfplots`, `tcolorbox` used sparingly for key results or safety notes.
- Title page modeled on PLUME: institution and sponsor logos at the top (only logos the user supplies), project or team name, horizontal rules framing the title, subtitle in the accent color, document type and "submitted for" line, institution/sponsor blocks.

**Color palette** (taken from the PLUME document, use consistently across headings, diagrams and plots)

| Role | Hex |
|---|---|
| Primary (headings, title, links) | `#143C6E` navy |
| Secondary (subheadings, main plot series) | `#43638B` steel blue |
| Tertiary series | `#4C87AD` light blue |
| Accent (highlights, second series, warnings) | `#CC7451` terracotta |
| Diagram fill light | `#CCCCFF` periwinkle (use at reduced opacity) |
| Diagram fill warm | `#F5E6B2` sand |
| Text / neutral lines | `#222222` / `#888888` |

Use the palette in matplotlib (custom style: serif font matching the document, light gray grid, palette as the color cycle, no top/right spines) and in TikZ (`\definecolor`). If the user's template has its own colors, use those instead.

**Layout rules**
- Figures placed near their first reference; no page with only a float unless it is a full-page figure.
- Every figure, diagram and table stays inside the section (and subsection) where it is written: it may never appear after the next heading, or above its own heading. In LaTeX, enforce this in the preamble, not by hand-placing floats:
  ```latex
  \usepackage{flafter}               % a float never appears before the point where it is written
  \usepackage[section]{placeins}     % \FloatBarrier at every \section
  \usepackage{etoolbox}
  \pretocmd{\subsection}{\FloatBarrier}{}{}
  \pretocmd{\subsubsection}{\FloatBarrier}{}{}
  ```
  In Word, insert figures in line with text (not floating) inside their section, with the caption kept with the figure. Check this on every page in the render check (section 7, step 8).
- No overfull boxes, widowed headings, or tables split badly across pages.
- Equations centered and numbered on the right.
- Write cross-references in words with `~\ref{}`: `equation~\ref{eq:thrust}`, `Figure~\ref{fig:fbd}`, `Table~\ref{tab:exp}`, `section~\ref{sec:model}`. Do not use `\eqref`-style "(18)" alone, "Eq."/"Fig." abbreviations, or cleveref defaults.
- Consistent spacing; no decorative clutter.

## 7. Build and check (results go to chat)

1. For long reports, agree the outline and visuals plan in chat first, then write section by section.
2. **Requirement check** against the checklist; page/word count verified by counting the compiled output.
3. **Consistency check:** abstract, results and conclusion use the same numbers; procedure matches the results; every figure, table, equation and section reference resolves; units are correct.
4. **Voice check:** pick at least six paragraphs spread across the document (including one derivation, one experimental/method paragraph and one results/discussion paragraph). For each, compare with the closest reference passage in section 3A and go through every row of the table in section 3B. Any paragraph that reads like a journal paper or textbook (terse, impersonal, imperative derivation steps, abbreviated references, no motivation or meaning sentences) is rewritten, and the check is repeated on new paragraphs until all pass. Also search the source for `Eq.`, `Eqs.`, `Fig.`, `\paragraph`, imperative derivation openers (`Differentiate`, `Define`, `First,`, `Next,`), and British spellings; fix every hit.
5. **Pattern pass:** search for every section-4 pattern and rewrite offending sentences.
6. **Critique loop:** privately critique anything still generic or artificial, fix it, then produce the final version.
7. **Two-channel scan:** search for placeholders, gap notes, references to the user/conversation, drafting comments, provenance tags. Resolve every hit; anything unresolved becomes a question in chat.
8. **Render check:** compile the document, rasterize the pages, and look at every page: figure legibility, palette consistency, float placement (every figure and table inside its own section, see section 6), overflow, title page, headers. Fix and recompile.
9. **Fresh-reader test** (procedures and anything others must follow): if subagents are available, give a fresh agent only the report and ask for steps it could not execute, terms or values it could not trace, and the main result in one sentence. Otherwise do this pass yourself as a newcomer. Fix what it finds.
10. Deliver: the report files, then a short chat note with checklist status, figure sources to confirm, anything narrowed or omitted and why, and open questions.

## Before / after

Before (AI voice): "The innovative nozzle design leverages compressible-flow principles to deliver robust thrust, highlighting the importance of careful throat sizing."

After (Nedal's voice): "Since the tank pressure remains above 1.92 bar for most of the discharge (see inequality 10), the flow through the throat is choked, and the mass flow rate becomes directly proportional to the air mass remaining in the tank (equation 18). Equation 24 shows that this mass decays exponentially, so the thrust is expected to be highest at release and to fall off quickly. Thus, the throat diameter sets both the peak thrust and how long it lasts."