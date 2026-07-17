#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "track_robot_semantic_memory/association_confirmation.hpp"
#include "track_robot_semantic_memory/appearance_memory.hpp"
#include "track_robot_semantic_memory/id_types.hpp"
#include "track_robot_semantic_memory/lifecycle_policy.hpp"
#include "track_robot_semantic_memory/memory_clock.hpp"
#include "track_robot_semantic_memory/memory_domain.hpp"
#include "track_robot_semantic_memory/motion_classifier.hpp"
#include "track_robot_semantic_memory/reidentification.hpp"
#include "track_robot_semantic_memory/types.hpp"

namespace track_robot_semantic_memory
{

struct LidarObservation
{
  ProducerObjectKey source_key;
  std::int64_t source_stamp_ns{0};
  std::array<double, 3> position{};
  std::array<double, 3> velocity{};
  std::array<double, 3> extent{};
  std::array<double, 9> position_covariance{};
  double confidence{0.0};

  void validate() const;
};

struct MemoryHistorySample
{
  std::int64_t source_stamp_ns{0};
  std::array<double, 3> position{};
  SupportState support{SupportState::kNone};
  LifecycleState lifecycle{LifecycleState::kTentative};
};

struct MemorySemanticEvidence
{
  std::string label;
  double confidence{0.0};
  std::string provenance;
  std::uint8_t evidence_kind{0U};
  std::uint64_t source_observation_id{0U};
};

struct VisualMemorySupplement
{
  ProducerObjectKey lidar_key;
  VisualAssociationKey visual_key;
  std::uint64_t observation_producer_epoch_id{0U};
  std::uint64_t observation_id{0U};
  std::uint64_t visual_candidate_id{0U};
  std::int64_t camera_stamp_ns{0};
  double association_confidence{0.0};
  bool association_confirmed{false};
  bool ambiguous{false};
  bool semantic_evidence_valid{false};
  bool appearance_evidence_valid{false};
  std::optional<AppearanceDescriptor> appearance_descriptor;
  double appearance_quality{0.0};
  bool prediction_only{false};
  std::vector<MemorySemanticEvidence> semantic_labels;

  void validate() const;
};

struct MemoryObject
{
  GlobalObjectKey key;
  ProducerObjectKey lidar_key;
  std::array<double, 3> position{};
  std::array<double, 3> velocity{};
  std::array<double, 3> extent{};
  std::array<double, 9> position_covariance{};
  LifecycleState lifecycle{LifecycleState::kTentative};
  SupportState support{SupportState::kNone};
  VisibilityState visibility{VisibilityState::kUnknown};
  MotionState motion{MotionState::kUncertain};
  std::int64_t first_seen_ns{0};
  std::int64_t last_seen_ns{0};
  std::int64_t state_stamp_ns{0};
  std::uint32_t compatible_hit_count{0U};
  std::uint32_t observation_count{0U};
  double confidence{0.0};
  std::optional<VisualAssociationKey> attached_visual_key;
  std::uint64_t visual_observation_producer_epoch_id{0U};
  std::uint64_t visual_observation_id{0U};
  std::uint64_t visual_candidate_id{0U};
  std::int64_t last_camera_seen_ns{-1};
  std::uint32_t camera_observation_count{0U};
  std::uint32_t semantic_update_count{0U};
  std::uint32_t appearance_update_count{0U};
  std::string appearance_summary_id;
  std::uint8_t appearance_prototype_count{0U};
  std::string appearance_encoder_id;
  std::string appearance_checkpoint_id;
  std::uint32_t appearance_descriptor_version{0U};
  ReidentificationState reidentification_state{ReidentificationState::kNotRequired};
  double association_confidence{0.0};
  std::vector<MemorySemanticEvidence> semantic_labels;
  std::vector<MemoryHistorySample> short_history;
};

enum class MemoryEventType
{
  kCreated,
  kConfirmed,
  kLifecycleChanged,
  kLost,
  kArchived,
  kDomainChanged,
  kMemoryReset,
  kObservationRejected,
  kCapacityEvicted,
  kAssociationAttached,
  kAssociationDetached,
  kReidentified,
  kInspectionChanged,
};

struct MemoryEvent
{
  MemoryEventType type{MemoryEventType::kCreated};
  GlobalObjectKey object_key;
  LifecycleState previous_lifecycle{LifecycleState::kTentative};
  LifecycleState current_lifecycle{LifecycleState::kTentative};
};

struct MemoryCoreConfig
{
  std::size_t max_objects{256U};
  std::size_t max_history{16U};
  std::size_t max_feature_prototypes{4U};
  double appearance_minimum_quality{0.5};
  double appearance_new_prototype_similarity_threshold{0.8};
  double appearance_normalization_tolerance{1e-4};
  std::int64_t rollback_tolerance_ns{1000000};
  LifecyclePolicyConfig static_lifecycle{};
  LifecyclePolicyConfig dynamic_lifecycle{3U, 300000000, 1000000000, 5000000000};
  double static_process_noise{0.01};
  double dynamic_process_noise{0.10};
  double static_max_speed_mps{0.15};
  double dynamic_min_speed_mps{0.35};

  void validate() const;
};

struct MemoryUpdateResult
{
  std::uint64_t memory_epoch_id{0U};
  std::vector<MemoryObject> objects;
  std::vector<MemoryObject> active_objects;
  std::vector<MemoryEvent> events;
  std::uint32_t rejected_observations{0U};
};

struct CurrentAppearanceEvidence
{
  GlobalObjectKey object_key;
  AppearanceDescriptor descriptor;
  std::int64_t camera_stamp_ns{0};
  std::uint64_t observation_producer_epoch_id{0U};
  std::uint64_t observation_id{0U};
  std::uint32_t appearance_update_count{0U};
};

struct VisualSupplementResult
{
  bool accepted{false};
  bool appearance_accepted{false};
  std::string reason;
  std::string appearance_reason;
  std::optional<CurrentAppearanceEvidence> current_appearance_evidence;
  MemoryUpdateResult snapshot;
};

struct ReidentificationStateUpdate
{
  GlobalObjectKey key;
  ReidentificationState state{ReidentificationState::kNotRequired};
};

struct ReidentificationTransferResult
{
  bool accepted{false};
  std::string reason;
  GlobalObjectKey preserved_key;
  MemoryUpdateResult snapshot;
};

class MemoryCore
{
public:
  MemoryCore(MemoryCoreConfig config, std::uint64_t initial_memory_epoch_id);

  MemoryUpdateResult update(
    const MemoryDomainKey & domain,
    std::int64_t batch_stamp_ns,
    std::vector<LidarObservation> observations);

  MemoryUpdateResult reset(const MemoryDomainKey & domain);

  VisualSupplementResult supplement_visual(
    const MemoryDomainKey & domain,
    const VisualMemorySupplement & supplement);

  [[nodiscard]] RuntimeReidentificationFrame make_reidentification_frame(
    const MemoryDomainKey & domain,
    std::uint64_t frame_index,
    const ReidentificationConfig & config,
    const std::vector<CurrentAppearanceEvidence> & current_appearance_evidence) const;

  [[nodiscard]] const std::vector<AppearancePrototype> * appearance_prototypes(
    const GlobalObjectKey & key) const noexcept;

  MemoryUpdateResult apply_reidentification_states(
    const MemoryDomainKey & domain,
    const std::vector<ReidentificationStateUpdate> & updates);

  ReidentificationTransferResult reidentify(
    const MemoryDomainKey & domain,
    const GlobalObjectKey & old_key,
    const GlobalObjectKey & replacement_key,
    const ProducerObjectKey & expected_replacement_lidar_key,
    const std::optional<VisualAssociationKey> & expected_replacement_visual_key);

private:
  void clear_for_new_epoch();
  MemoryObject * find_by_key(const GlobalObjectKey & key);
  MemoryObject * find_by_source(const ProducerObjectKey & key);
  MemoryObject * find_by_visual(const VisualAssociationKey & key);
  bool make_capacity(MemoryUpdateResult & result);
  MemoryObject & create_object(
    const LidarObservation & observation, MemoryUpdateResult & result);
  void apply_observation(
    MemoryObject & object, const LidarObservation & observation,
    MemoryUpdateResult & result);
  void advance_unobserved(
    std::int64_t batch_stamp_ns,
    const std::vector<ProducerObjectKey> & observed_keys,
    MemoryUpdateResult & result);
  void transition_lifecycle(
    MemoryObject & object, LifecycleState next, MemoryUpdateResult & result);
  void append_history(MemoryObject & object);
  void refresh_appearance_summary(MemoryObject & object);
  MemoryUpdateResult finish_result(MemoryUpdateResult result) const;

  MemoryCoreConfig config_;
  MemoryDomainTracker domain_tracker_;
  MemoryClock clock_;
  LifecyclePolicy static_lifecycle_;
  LifecyclePolicy dynamic_lifecycle_;
  MotionClassifier motion_classifier_;
  std::uint64_t next_global_object_id_{1U};
  std::vector<MemoryObject> objects_;
  std::map<ProducerObjectKey, GlobalObjectKey> source_index_;
  std::map<GlobalObjectKey, AppearanceMemory> appearance_banks_;
};

}  // namespace track_robot_semantic_memory
