/**
 * rm-guard — OpenCode 插件：把 autoaim 的回归门禁暴露成一个工具。
 *
 * 作用：任何一次改完算法/参数后，代理可以直接调用 `rm_gate` 跑
 * `tools/regression.py`（两条真实视频 + 仿真，对比 BASELINE §10 的阈值），
 * 拿到 PASS/FAIL 与逐项 delta，而不是靠"看起来对"。
 *
 * 安装（二选一）：
 *   A) 项目级：把本目录复制到  <repo>/.opencode/plugins/rm-guard/  后自动加载
 *   B) 全局：在 ~/.config/opencode/opencode.json 的 plugins 里加绝对路径
 *
 * 环境变量 RM_REPO 指定仓库根目录（默认见下）。
 */
import { Plugin } from "@opencode/plugin"
import { execFile } from "node:child_process"
import { promisify } from "node:util"

const exec = promisify(execFile)
const REPO = process.env.RM_REPO ?? "C:/Users/Lgnatius/Desktop/RM/autoaim-takeover"

export default Plugin.define({
  id: "rm-guard",
  async setup(ctx) {
    await ctx.tool.transform((editor) => {
      editor.namespace({
        name: "rm",
        description: "RoboMaster autoaim regression tools",
      })

      editor.add({
        name: "gate",
        description:
          "Run the autoaim regression gate: two real videos plus the simulator, " +
          "compared against the thresholds in BASELINE.md section 10. " +
          "Returns PASS/FAIL and the per-metric delta. Use it before claiming a change is done.",
        input: {
          type: "object",
          properties: {
            quick: {
              type: "boolean",
              description: "Skip the simulator (faster, videos only)",
            },
          },
          additionalProperties: false,
        },
        execute: async (input, context) => {
          const args = ["tools/regression.py"]
          if ((input as { quick?: boolean } | undefined)?.quick) args.push("--quick")
          try {
            const { stdout } = await exec("python", args, {
              cwd: REPO,
              timeout: 20 * 60 * 1000,
              maxBuffer: 4 * 1024 * 1024,
              signal: context.signal,
            })
            return { content: stdout.slice(-6000) }
          } catch (err) {
            const e = err as { stdout?: string; stderr?: string; message?: string }
            const text = `${e.stdout ?? ""}\n${e.stderr ?? e.message ?? ""}`.trim()
            return { content: `REGRESSION GATE FAILED\n${text.slice(-6000)}` }
          }
        },
      })
    })
  },
})
